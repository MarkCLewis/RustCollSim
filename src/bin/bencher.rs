use std::time::Instant;

use rust_coll_sim::{simulation_setups::demo_big_sim_hills_sliding_brick, Opts, RingType};

fn opts_gen(
    particles: usize,
    disable_pq: bool,
    ring_type: RingType,
    serialize: bool,
    seed: u64,
) -> Opts {
    Opts {
        particles,
        no_warnings: true,
        seed, //44 is good for B1k, B10k // 2 is bad for B1k
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
    }
}

fn seed_picker(particles: usize, ring_type: RingType, disable_pq: bool) -> Option<u64> {
    // if disable_pq {
    //     return Some(42);
    // }

    // finding non-exploding seeds
    match (particles, ring_type) {
        (1_000, RingType::B) => Some(44),
        (10_000, RingType::B) => Some(44),
        (1_000, RingType::A) => Some(47),
        (10_000, RingType::A) => None, // 16,18,19,25,36,43,47 meh // 22 meh+
        (_, _) => panic!("no seed for this combination: {} {}", particles, ring_type),
    }
}

fn main() {
    //1M, 1k, 10k, 100k
    // no pq for 10_000 - do not consider pq
    // pq for 1000

    let serialize = false;

    let sim_sizes = [1_000, 10_000]; //[1_000, 10_000]; //, 100_000, 1_000_000]; // [1000, 10000, 100000, 1000000];

    for ring_type in [RingType::A, RingType::B] {
        for disable_pq in [false, true] {
            for sim in sim_sizes {
                let Some(seed) = seed_picker(sim, ring_type, disable_pq) else { continue };

                let opts = opts_gen(sim, disable_pq, ring_type, serialize, seed);
                let pre = Instant::now();
                demo_big_sim_hills_sliding_brick(opts); // demo_big_sim_hills_no_sliding_brick
                let post = Instant::now();
                eprintln!("runtime = {}", (post - pre).as_secs_f64());
                eprintln!(" ");
            }
        }
    }
}
