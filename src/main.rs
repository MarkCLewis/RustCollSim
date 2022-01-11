#![feature(portable_simd)]

mod particle;
mod simd_particle;
// mod kd_tree1;

// use kd_tree1::KDTree;

fn main() {
    println!("Hello, collisional simulations!");

    particle::simple_sim();

    simd_particle::simple_sim();
}
