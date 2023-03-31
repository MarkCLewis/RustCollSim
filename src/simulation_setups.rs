use std::{f64::consts::PI, fs::File};

use indicatif::ProgressBar;

use crate::{
    debugln, hills_force, particle::Particle, system::KDTreeSystem, util::overlap_grid::generate,
    vectors::Vector, Opts,
};

fn cell_size(particles: usize, cell_density: f64) -> f64 {
    let area = particles as f64 / cell_density;
    area.sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_cell_size() {
        assert_eq!(cell_size(100, 1e12), 1e-5);
    }
}

pub fn demo_big_sim_hills_sliding_brick(opts: Opts) {
    fastrand::seed(opts.seed);

    let dt = 2. * PI / opts.steps_in_2pi as f64;

    // density = particles / area
    // density * area = particles
    // area = particles / density
    // cell_size = sqrt(area)

    let cell_size = cell_size(opts.particles, opts.cell_density);

    let cell = hills_force::SlidingBrickBoundary::new(cell_size, cell_size);

    let r = 1e-7;
    let rho = 0.88;

    // range -0.5..0.5
    let centered_rand = || fastrand::f64() - 0.5;

    let particle_gen = || {
        let p = Vector::new(
            centered_rand() * cell.sx,
            centered_rand() * cell.sy,
            centered_rand() * 2. * r, // +- 1e-7
        );

        let v = Vector::new(0., -1.5 * p.x(), 0.);

        Particle::new(p, v, r, rho)
    };

    let pop = generate(opts.particles, particle_gen, r);

    eprintln!("pop created: {}", pop.len());

    eprintln!("dt: {}", dt);

    eprintln!("cell size: {}", cell_size);

    // this is only for plotting
    debugln!("SETUP r0={}, r1={}, rho={}, init_impact_v={}, sep_dis={}, dt={}, steps={}, desired_steps={}", r, r, rho, 0, 0, 0, 0, 0);

    let mut sys = KDTreeSystem::new(pop, dt, 15, 0.5, Some(&opts))
        .set_hills_force(hills_force::HillsForce::new())
        .set_sliding_brick(cell)
        .set_serialize_run(match opts.particles_file.as_str().trim() {
            "" => None,
            file => Some(File::create(file).unwrap()),
        })
        .set_progress_bar(Some(ProgressBar::new(opts.big_steps as u64)))
        .set_disable_pq(opts.disable_pq);

    sys.run(opts.big_steps); // 1000
}

/*
if !opts.no_serialize {
            Some(File::create("demo_big_sim_hills_sliding_brick.csv").unwrap())
        } else {
            None
        } */
