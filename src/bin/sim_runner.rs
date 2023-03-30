use rust_coll_sim::{simulation_setups::demo_big_sim_hills_sliding_brick, Opts};

fn main() {
    //1M, 1k, 10k, 100k
    // no pq for 10_000 - do not consider pq
    // pq for 1000

    let opts_gen = |particles, disable_pq| Opts {
        particles,
        no_warnings: false,
        seed: 42,
        steps_in_2pi: 1000,
        big_steps: 1000,
        cell_density: 1e12,
        particles_file: format!("data/big_sim_hills_sliding_brick_{}.csv", particles),
        disable_pq,
    };

    let sim_sizes = [1000]; // [1000, 10000, 100000, 1000000];

    for sim in sim_sizes {
        let opts = opts_gen(sim, false);
        demo_big_sim_hills_sliding_brick(opts);
    }
}
