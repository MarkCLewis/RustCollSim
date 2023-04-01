use rust_coll_sim::{simulation_setups::demo_big_sim_hills_sliding_brick, Opts};

fn main() {
    //1M, 1k, 10k, 100k
    // no pq for 10_000 - do not consider pq
    // pq for 1000

    let opts_gen = |particles, disable_pq| Opts {
        particles,
        no_warnings: true,
        seed: 42, //43
        steps_in_2pi: if disable_pq { 10_000 } else { 1000 },
        big_steps: if disable_pq { 10_000 } else { 1000 },
        cell_density: 1e12,
        particles_file: format!(
            "data/big_sim_hills_sliding_brick_{}_disable_pq={disable_pq}.csv",
            particles
        ),
        disable_pq,
    };

    let sim_sizes = [1_000]; // [1000, 10000, 100000, 1000000];

    for disable_pq in [false, true] {
        for sim in sim_sizes {
            let opts = opts_gen(sim, disable_pq);
            demo_big_sim_hills_sliding_brick(opts);
            eprintln!(" ");
        }
    }
}
