#![feature(portable_simd)]

mod particle;
mod simd_particle;
mod kd_tree2;

use std::time::Instant;

fn main() {
    println!("Hello, collisional simulations!");

    let dt = 1e-3 * 2.0 * std::f64::consts::PI;

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
        kd_tree2::simple_sim(&mut simd_particle::circular_orbits(10000), dt);
        println!("{}", start.elapsed().as_nanos());
    }
}
