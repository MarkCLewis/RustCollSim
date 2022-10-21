#![feature(portable_simd)]

mod kd_tree;
mod particle;
mod short_range_forcing;
mod soft_collision_queue;
mod util;

use std::time::Instant;

fn main() {
    println!("Hello, collisional simulations!");

    let dt = 1e-3; // * 2.0 * std::f64::consts::PI;

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

    {
        let start = Instant::now();
        // kd_tree2::simple_sim(&mut simd_particle::two_bodies(), dt);
        kd_tree::simple_sim(&mut particle::circular_orbits(20001), dt, 6281);
        // kd_tree2::simple_sim(&mut &mut simd_particle::galactic_orbits(20001), dt, 6281);
        println!("{}", start.elapsed().as_nanos() as f64 / 1e9);
    }
}
