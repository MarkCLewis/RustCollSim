#![feature(portable_simd)]
#![feature(hash_drain_filter)]

mod debug;
mod impact_vel_tracker;
mod kd_tree;
mod no_explode;
mod particle;
mod soft_collision_queue;
mod system;
mod tests;
mod util;
mod vectors;

use std::{f64::consts::PI, time::Instant};

use crate::{no_explode::rotter, system::KDTreeSystem};

fn main() {
    println!("Hello, collisional simulations!");

    demo2();
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

fn demo2() {
    let dt = 1e-3;

    let r = 1e-7;
    let rho = 0.88;
    let init_impact_v = 2. * r * 20.;
    let sep_dis = 2.2 * r; // x = 1.1r

    let mut sys = KDTreeSystem::new(
        particle::two_equal_bodies(r, rho, init_impact_v, sep_dis),
        dt,
    );

    sys.run(11); // 250
}

fn demo1() {
    let dt = 1e-3;

    {
        let start = Instant::now();

        // let mut sys = KDTreeSystem::new(particle::circular_orbits(20001), dt);
        let mut sys = KDTreeSystem::new(particle::two_bodies(), dt);

        sys.run((2. * PI / dt) as usize);

        println!("{}", start.elapsed().as_nanos() as f64 / 1e9);
    }
}
