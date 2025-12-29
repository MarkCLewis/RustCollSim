#![allow(unused_parens)] // this is an issue with clap where removing the parens causes a macro issue
#![feature(portable_simd)]
//#![feature(hash_drain_filter)]
//#![feature(core_intrinsics)]

use std::fmt::Display;
use std::str::FromStr;


use clap::Parser;

pub mod debug;
pub mod debug_tools;
pub mod hills_force;
pub mod impact_vel_tracker;
pub mod kd_tree;
pub mod no_explode;
pub mod particle;
pub mod simulation_setups;
pub mod soft_collision_queue;
pub mod system;
pub mod tests;
pub mod util;
pub mod design;
pub mod boundary_conditions;
pub mod vectors;


#[derive(Parser, Debug)]
pub struct Opts {
    /// number of particles
    #[clap(short, long, default_value_t = 100)]
    pub particles: usize,
    /// prevents printing warnings about small dt (for speed / keeping output clean)
    #[clap(short, long, default_value_t = false)]
    pub no_warnings: bool,
    /// RNG seed
    #[clap(long, default_value_t = 42)]
    pub seed: u64,
    /// number of steps in 2PI
    #[clap(long, default_value_t = 1000)]
    pub steps_in_2pi: usize,
    /// number of big time steps
    #[clap(long, default_value_t = 1000)]
    pub big_steps: usize,
    /// size of cell for sliding brick
    #[clap(long, default_value_t = 1e12)]
    pub cell_density: f64,
    /// file name to save particles for movie plotting
    /// set empty string to prevent serializing (for speed)
    #[clap(long, default_value_t = ("".into()))] //("demo_big_sim_hills_sliding_brick.csv".into()))]
    pub particles_file: String,
    #[clap(long, default_value_t = false)]
    pub disable_pq: bool,
    #[clap(long, default_value_t = RingType::default(), 
        value_parser = clap::value_parser!(RingType))]
    pub ring_type: RingType,
}


#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum RingType {
    A,
    B,
}

impl RingType {
    pub fn get_rho(&self) -> f64 {
        match self {
            RingType::A => 1.9,
            RingType::B => 0.88,
        }
    }
}

impl Default for RingType {
    fn default() -> Self {
        RingType::B
    }
}

impl Display for RingType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RingType::A => write!(f, "A"),
            RingType::B => write!(f, "B"),
        }
    }
}

impl FromStr for RingType {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "A" => Ok(RingType::A),
            "B" => Ok(RingType::B),
            _ => Err(format!("{} is not a valid RingType", s)),
        }
    }
}
