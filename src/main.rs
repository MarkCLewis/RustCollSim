use clap::Parser;
use rust_coll_sim::{simulation_setups::demo_big_sim_hills_sliding_brick, Opts};

fn main() {
    println!("Hello, collisional simulations!");
    let opts = Opts::parse();

    if cfg!(feature = "no_gravity") {
        eprintln!("Running with feature: no_gravity");
    }

    if cfg!(feature = "early_quit") {
        eprintln!("Running with feature: early_quit");
    }

    demo_big_sim_hills_sliding_brick(opts);
}
