#![feature(portable_simd)]
#![feature(hash_drain_filter)]

mod impact_vel_tracker;
mod kd_tree;
mod no_explode;
mod particle;
mod soft_collision_queue;
mod system;
mod util;
mod vectors;

use std::time::Instant;

use crate::{no_explode::rotter, system::KDTreeSystem};

fn main() {
    println!("Hello, collisional simulations!");

    demo1();
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

fn demo1() {
    let dt = 1e-3;

    {
        let start = Instant::now();

        // let mut sys = KDTreeSystem::new(particle::circular_orbits(20001), dt);
        let mut sys = KDTreeSystem::new(particle::two_bodies(), dt);

        for i in 0 as usize..10 {
            //6281
            println!("step: {}", i);
            sys.apply_forces(i);
            sys.end_step(i);

            if i % 10 == 0 {
                sys.pq.borrow_mut().trim_impact_vel_tracker(i);
            }
        }

        println!("{}", start.elapsed().as_nanos() as f64 / 1e9);
    }
}
