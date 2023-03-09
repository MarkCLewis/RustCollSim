#[cfg(test)]
pub mod test_collection {
    use anyhow::ensure;

    use crate::test_setup::test_setup::{write_to_csv, TestSetup};

    #[test]
    /// This test is a utility for debugging a single test case from a run,
    /// by copying and pasting the line from the csv file.
    pub fn debug_one_test() -> anyhow::Result<()> {
        ensure!(
            cfg!(feature = "early_quit"),
            "early_quit feature is required for this test"
        );
        ensure!(
            cfg!(feature = "no_gravity"),
            "no_gravity feature is required for this test"
        );

        // copied and pasted from the csv file
        let config =
          "0.0000001,0.00000003,0.000001,1,0.88,0.5236928968338461,0.30850906195856287,1.0283635398618762,14,0.000001,15,0.00000014300000000000002,1,0.5            ";
        let cols = config.trim().split(',').collect::<Vec<_>>();

        match &cols[..] {
            [radius_0, radius_1, desired_impact_vel, time_step, rho, _coeff_of_res, _max_pen_depth_percent_0, _max_pen_depth_percent_1, _collision_steps, _real_impact_vel, desired_collision_step_count, initial_separation_distance, simulation_max_step_count, desired_coeff_of_res] =>
            {
                let radius_0 = radius_0.parse::<f64>()?;
                let radius_1 = radius_1.parse::<f64>()?;
                let desired_impact_vel = desired_impact_vel.parse::<f64>()?;
                let time_step = time_step.parse::<f64>()?;
                let rho = rho.parse::<f64>()?;
                let desired_collision_step_count = desired_collision_step_count.parse::<usize>()?;
                let initial_separation_distance = initial_separation_distance.parse::<f64>()?;
                let simulation_max_step_count = simulation_max_step_count.parse::<usize>()?;
                let desired_coeff_of_res = desired_coeff_of_res.parse::<f64>()?;

                let setup = TestSetup::new(
                    time_step,
                    radius_0,
                    radius_1,
                    rho,
                    desired_impact_vel,
                    initial_separation_distance,
                    simulation_max_step_count,
                    desired_collision_step_count,
                    desired_coeff_of_res,
                );

                match setup.run() {
                    Ok(result) => {
                        println!("result: {}", result);
                    }
                    Err(e) => {
                        println!("error: {:?}", e);
                    }
                }
            }
            _ => panic!(
                "Invalid config, wrong number of columns, got {}",
                cols.len()
            ),
        }

        Ok(())
    }

    #[test]
    pub fn generate_and_run_setups_0() -> anyhow::Result<()> {
        ensure!(
            cfg!(feature = "early_quit"),
            "early_quit feature is required for this test"
        );
        // ensure!(
        //     cfg!(feature = "no_gravity"),
        //     "no_gravity feature is required for this test"
        // );

        let dt_vals = [1., 0.5, 0.1, 0.05, 0.025];
        let velocities_vals = [3e-8, 1e-7, 3e-7, 1e-6, 3e-6];

        let r1_vals = [1e-7, 3e-8, 1e-8];
        // r2 should be less than or equal to r1
        let r2_producer = |r1: f64| r1_vals.iter().filter(move |&&r2| r2 <= r1);

        let rho = 0.88;
        // let init_impact_v = 2. * r;
        let sep_dis_producer = |r1: f64, r2: f64| 1.1 * (r1 + r2); // x = 1.1r

        // dt min is so that dt * desired_steps = full collision time

        let desired_steps_vals = [1, 3, 5, 7, 10, 12, 15, 20];

        let desired_coeff_of_res_vals = [0.2, 0.5, 0.7, 0.9]; //[0.2, 0.5, 0.7, 1.];

        // steps = 0.25/dt at v=2e-7.
        // so if v=1e-7, the collision will take twice as much time.
        // T(2e-7) = 0.25, T(1e-7) = 0.5, T(v) = 1e-7*0.5/v
        let steps = |dt: f64, v: f64| ((1e-7 * 0.5 / v) / dt) as usize;

        let mut results = Vec::new();

        for desired_coeff_of_res in desired_coeff_of_res_vals {
            for dt in dt_vals {
                for r1 in r1_vals {
                    for &r2 in r2_producer(r1) {
                        for init_impact_v in velocities_vals {
                            for desired_steps in desired_steps_vals {
                                let sep_dis = sep_dis_producer(r1, r2);
                                let steps = steps(dt, init_impact_v);
                                let setup = TestSetup::new(
                                    dt,
                                    r1,
                                    r2,
                                    rho,
                                    init_impact_v,
                                    sep_dis,
                                    steps,
                                    desired_steps,
                                    desired_coeff_of_res,
                                );
                                results.push((setup.run(), setup));
                            }
                        }
                    }
                }
            }
        }
        eprintln!("test count: {}", results.len());
        eprintln!(
            "test success: {} / {}",
            results.iter().filter(|(r, _)| r.is_ok()).count(),
            results.len()
        );

        let file_name = if cfg!(feature = "no_gravity") {
            "data/generate_and_run_setups_0.csv"
        } else {
            "data/generate_and_run_setups_0_gravity.csv"
        };

        eprintln!("writing to csv, {}", file_name,);

        write_to_csv(file_name, results)?;

        Ok(())
    }
}
