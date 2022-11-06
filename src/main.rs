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

use crate::{no_explode::rotter, particle::Particle, system::KDTreeSystem, vectors::Vector};

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

        fn pair_force(p1: &mut Particle, p2: &mut Particle) -> (Vector, Vector) {
            let x_len = (Vector(p1.p) - Vector(p2.p)).mag();
            let x_hat = (Vector(p1.p) - Vector(p2.p)) / x_len;
            let separation_dis = x_len - p1.r - p2.r;
            let vji = Vector(p1.p) - Vector(p2.p);
            // FIXME: impact vel tracker
            let impact_vel = 0.;

            if separation_dis < 0. {
                // colliding
                let reduced_mass = (p1.m * p2.m) / (p1.m + p2.m);
                let (b, k) = rotter::b_and_k(impact_vel, reduced_mass, f64::min(p1.r, p2.r));

                let f_spring = x_hat * -k * separation_dis;
                let f_damp = vji * -b;

                let f_total = f_spring + f_damp;

                return (f_total / p1.m, f_total / p2.m);
            } else {
                // not colliding
                // FIXME: gravity? calc_pp_accel
                return (Vector::ZERO, Vector::ZERO);
            }
        }

        let mut sys = KDTreeSystem::new(particle::circular_orbits(20001), dt, pair_force);

        for i in 0 as usize..10 {
            //6281
            println!("step: {}", i);
            sys.apply_forces(i);
            sys.end_step(i);

            if i % 10 == 0 {
                sys.trim_impact_vel_tracker(i);
            }
        }

        println!("{}", start.elapsed().as_nanos() as f64 / 1e9);
    }
}
