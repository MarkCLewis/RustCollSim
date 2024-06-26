use std::{f64::consts::PI, fs::File};

use indicatif::{ProgressBar, ProgressStyle};

use crate::{
    debugln, hills_force,
    particle::Particle,
    system::KDTreeSystem,
    util::{overlap_grid::generate, progress_tracker::TwoPartProgress},
    vectors::Vector,
    Opts,
};

fn cell_size(particles: usize, cell_density: f64) -> f64 {
    let area = particles as f64 / cell_density;
    area.sqrt()
}

/// sx = aspect_ratio * sy
/// returns (sx, sy)
fn cell_size_rect(particles: usize, cell_density: f64, aspect_ratio: f64) -> (f64, f64) {
    let area = particles as f64 / cell_density;

    // sx * sy = area
    // sx = ar * sy
    // thus
    // ar * sy * sy = area
    // sy = sqrt(area / ar)
    let sy = (area / aspect_ratio).sqrt();
    let sx = area / sy;
    assert_eq!(sx * sy, area);
    assert!(((sx / sy) - aspect_ratio).abs() < f64::EPSILON);
    (sx, sy)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_cell_size() {
        assert_eq!(cell_size(100, 1e12), 1e-5);
    }
}

pub fn demo_big_sim_hills_no_sliding_brick(opts: Opts, do_bars: bool) {
    fastrand::seed(opts.seed);

    let dt = 2. * PI / opts.steps_in_2pi as f64;

    // density = particles / area
    // density * area = particles
    // area = particles / density
    // cell_size = sqrt(area)

    let aspect_ratio = match opts.particles {
        1_000 => 0.1,
        10_000 => 0.1,
        100_000 => 0.1,
        1_000_000 => 0.01,
        _ => panic!("unsupported particle count: {}", opts.particles),
    };

    let (sx, sy) = cell_size_rect(opts.particles, opts.cell_density, aspect_ratio);

    assert!(sx < sy);

    let r = 1e-7;
    // 0.88 in B ring, 1.9 in A ring, causes clumping
    let rho = opts.ring_type.get_rho();

    // range -0.5..0.5
    let centered_rand = || fastrand::f64() - 0.5;

    let particle_gen = || {
        let p = Vector::new(
            centered_rand() * sx,
            centered_rand() * sy,
            centered_rand() * 2. * r, // +- 1e-7
        );

        let v = Vector::new(0., -1.5 * p.x(), 0.);

        Particle::new(p, v, r, rho)
    };

    let pop = generate(opts.particles, particle_gen, r);

    eprintln!("pop created: {}", pop.len());

    eprintln!("dt: {}", dt);

    eprintln!("cell size: {} x {}", sx, sy);

    eprintln!("ring type: {}", opts.ring_type);

    // this is only for plotting
    debugln!("SETUP r0={}, r1={}, rho={}, init_impact_v={}, sep_dis={}, dt={}, steps={}, desired_steps={}", r, r, rho, 0, 0, 0, 0, 0);

    let bar = get_progress_bar(&opts);

    let mut sys = KDTreeSystem::new(pop, dt, 15, 0.5, Some(&opts))
        .set_hills_force(hills_force::HillsForce::new())
        .set_serialize_run(match opts.particles_file.as_str().trim() {
            "" => None,
            file => Some(File::create(file).unwrap()),
        })
        .set_progress_bar(if do_bars { Some(bar) } else { None })
        .set_disable_pq(opts.disable_pq);

    sys.run(opts.big_steps); // 1000
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
    // 0.88 in B ring, 1.9 in A ring, causes clumping
    let rho = opts.ring_type.get_rho();

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

    eprintln!("ring type: {}", opts.ring_type);

    // this is only for plotting
    debugln!("SETUP r0={}, r1={}, rho={}, init_impact_v={}, sep_dis={}, dt={}, steps={}, desired_steps={}", r, r, rho, 0, 0, 0, 0, 0);

    let _bar = get_progress_bar(&opts);

    let mut sys = KDTreeSystem::new(pop, dt, 15, 0.5, Some(&opts))
        .set_hills_force(hills_force::HillsForce::new())
        .set_sliding_brick(cell)
        .set_serialize_run(match opts.particles_file.as_str().trim() {
            "" => None,
            file => Some(File::create(file).unwrap()),
        })
        .set_progress_bar(/*Some(bar)*/ None)
        .set_disable_pq(opts.disable_pq);

    sys.run(opts.big_steps);
}

fn get_progress_bar(opts: &Opts) -> TwoPartProgress {
    // progress bar stuff
    let progress_bar = ProgressBar::new(opts.big_steps as u64);
    let main_sty =
        ProgressStyle::with_template("[{elapsed_precise}] {wide_bar:.blue/white} {pos:>7}/{len:7}")
            .unwrap();
    progress_bar.set_style(main_sty);

    let sub_sty = ProgressStyle::with_template("{wide_bar:.blue/white} {msg:30}").unwrap();
    let sub_bar = ProgressBar::new(6);
    sub_bar.set_style(sub_sty);

    let sub_sub_sty =
        ProgressStyle::with_template("{wide_bar:.blue/white} {msg:14} {pos:>7}/{len:7}").unwrap();
    let sub_sub_bar = ProgressBar::new(0);
    sub_sub_bar.set_style(sub_sub_sty);

    let bar = TwoPartProgress::triple(progress_bar, sub_bar, sub_sub_bar);
    bar
}

/*
if !opts.no_serialize {
            Some(File::create("demo_big_sim_hills_sliding_brick.csv").unwrap())
        } else {
            None
        } */
