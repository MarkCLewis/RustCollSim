#![feature(portable_simd)]
#![feature(hash_drain_filter)]

mod debug;
mod hills_force;
mod impact_vel_tracker;
mod kd_tree;
mod no_explode;
mod particle;
mod soft_collision_queue;
mod system;
mod test_collection;
mod test_setup;
mod unit_tests;
mod util;
mod vectors;

use std::{f64::consts::PI, fs::File, time::Instant};

use crate::{particle::Particle, system::KDTreeSystem, vectors::Vector};

fn main() {
    println!("Hello, collisional simulations!");
    if cfg!(feature = "no_gravity") {
        eprintln!("Running with feature: no_gravity");
    }

    if cfg!(feature = "early_quit") {
        eprintln!("Running with feature: early_quit");
    }

    demo_big_sim_hills_sliding_brick();
    return;

    // let dt = 1e-3; // * 2.0 * std::f64::consts::PI;

    // {
    //     let start = Instant::now();
    //     particle::simple_sim(particle::two_bodies(), dt);
    //     println!("{}", start.elapsed().as_nanos());
    // }

    // {
    //     let start = Instant::now();
    //     simd_particle::simple_sim(simd_particle::two_bodies(), dt);
    //     println!("{}", start.elapsed().as_nanos());
    // }

    // {
    //     let start = Instant::now();
    //     // kd_tree2::simple_sim(&mut simd_particle::two_bodies(), dt);
    //     kd_tree::simple_sim(&mut particle::circular_orbits(20001), dt, 6281);
    //     // kd_tree2::simple_sim(&mut &mut simd_particle::galactic_orbits(20001), dt, 6281);
    //     println!("{}", start.elapsed().as_nanos() as f64 / 1e9);
    // }
}

fn demo_big_sim_hills_sliding_brick() {
    fastrand::seed(42);

    let dt = 2. * PI / 1000.;

    let cell = hills_force::SlidingBrickBoundary::new(1e-5, 1e-5);

    let r = 1e-7;
    let rho = 0.88;

    // range -0.5..0.5
    let centered_rand = || fastrand::f64() - 0.5;

    let mut pop: Vec<Particle> = Vec::new();

    for _ in 0..100 {
        let mut p;
        loop {
            p = Vector::new(
                centered_rand() * cell.sx,
                centered_rand() * cell.sy,
                centered_rand() * 2. * r, // +- 1e-7
            );

            let is_overlap = pop
                .iter()
                .any(|p_old| (p - p_old.p).mag_sq() < (p_old.r + r) * (p_old.r + r));

            if !is_overlap {
                break;
            }
        }

        let v = Vector::new(0., -1.5 * p.x(), 0.);

        let particle = particle::Particle::new(p, v, r, rho);
        pop.push(particle);
    }

    eprintln!("pop created: {}", pop.len());

    let file = File::create("demo_big_sim_hills_sliding_brick.csv").unwrap();

    let mut sys = KDTreeSystem::new(pop, dt, 15, 0.5)
        .set_hills_force(hills_force::HillsForce::new())
        .set_sliding_brick(cell)
        .set_serialize_run(file);

    sys.run(1000); // 1000
}

#[allow(dead_code)]
fn demo1() {
    let dt = 1e-3;

    {
        let start = Instant::now();

        // let mut sys = KDTreeSystem::new(particle::circular_orbits(20001), dt);
        let mut sys = KDTreeSystem::new(particle::two_bodies(), dt, 10, 0.5);

        sys.run((2. * PI / dt) as usize);

        println!("{}", start.elapsed().as_nanos() as f64 / 1e9);
    }
}
