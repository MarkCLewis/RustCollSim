use std::time::Instant;

use rust_coll_sim::{simulation_setups::demo_big_sim_hills_sliding_brick, Opts, RingType};

fn main() {
    //1M, 1k, 10k, 100k
    // no pq for 10_000 - do not consider pq
    // pq for 1000

    let serialize = false;

    let opts_gen = |particles, disable_pq, ring_type| Opts {
        particles,
        no_warnings: true,
        seed: 44, //44 is good for B1k, B10k // 2 is bad for B1k
        steps_in_2pi: if disable_pq { 1_000 } else { 100 },
        big_steps: if disable_pq { 1_000 } else { 100 },
        cell_density: 1e12,
        particles_file: if serialize {
            format!(
            "data/big_sim_hills_sliding_brick_{}_disable_pq={disable_pq}_ring_type={ring_type}.csv",
            particles
        )
        } else {
            "".into()
        },
        disable_pq,
        ring_type,
    };

    let sim_sizes = [1_000, 10_000, 100_000, 1_000_000]; // [1000, 10000, 100000, 1000000];

    for ring_type in [RingType::B, RingType::A] {
        //RingType::A,
        for disable_pq in [false, true] {
            // if disable_pq {
            //     continue;
            // } // FIXME: remove this

            for sim in sim_sizes {
                let opts = opts_gen(sim, disable_pq, ring_type);
                let pre = Instant::now();
                demo_big_sim_hills_sliding_brick(opts);
                let post = Instant::now();
                eprintln!("runtime = {}", (post - pre).as_secs_f64());
                eprintln!(" ");
            }
        }
    }
}
