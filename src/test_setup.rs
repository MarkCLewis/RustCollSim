#[cfg(test)]
pub mod test_setup {
    use std::path::Path;

    use crate::{
        particle::{self, momentum, Particle, ParticleIndex},
        system::KDTreeSystem,
    };

    pub fn write_to_csv(
        file: impl AsRef<Path>,
        results: Vec<(anyhow::Result<TestResult>, TestSetup)>,
    ) -> anyhow::Result<()> {
        let mut writer = csv::Writer::from_path(file)?;
        writer.write_record([
            "radius_0",
            "radius_1",
            "desired_impact_vel",
            "time_step",
            "rho",
            "coeff_of_res",
            "max_pen_depth_percent_0",
            "max_pen_depth_percent_1",
            "collision_steps",
            "real_impact_vel",
            "desired_collision_step_count",
        ])?;

        for (result, setup) in results {
            // let result = result?;
            match result {
                Ok(result) => {
                    // writer.serialize(helper)?;
                    writer.write_record([
                        setup.r0.to_string(),
                        setup.r1.to_string(),
                        setup.init_impact_v.to_string(),
                        setup.dt.to_string(),
                        setup.rho.to_string(),
                        result.coeff_of_res.to_string(),
                        result.max_pen_depth_percent_1.to_string(),
                        result.max_pen_depth_percent_2.to_string(),
                        result.collision_step_count_actual.to_string(),
                        result.real_impact_vel.to_string(),
                        setup.desired_steps.to_string(),
                    ])?;
                }
                Err(e) => {
                    // writer.serialize(helper)?;
                    writer.write_record([
                        setup.r0.to_string(),
                        setup.r1.to_string(),
                        setup.init_impact_v.to_string(),
                        setup.dt.to_string(),
                        setup.rho.to_string(),
                        "error".to_string(),
                        e.to_string(),
                        "".to_string(),
                        "".to_string(),
                        "".to_string(),
                        setup.desired_steps.to_string(),
                    ])?;
                }
            }
        }

        Ok(())
    }

    #[test]
    pub fn generate_and_run_setups_0() -> anyhow::Result<()> {
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

        let steps = |dt: f64| (0.25 / dt) as usize;

        let mut results = Vec::new();

        let mut test_count = 0;

        for dt in dt_vals {
            for r1 in r1_vals {
                for &r2 in r2_producer(r1) {
                    for init_impact_v in velocities_vals {
                        for desired_steps in desired_steps_vals {
                            let sep_dis = sep_dis_producer(r1, r2);
                            let steps = steps(dt);
                            let setup = TestSetup::new(
                                dt,
                                r1,
                                r2,
                                rho,
                                init_impact_v,
                                sep_dis,
                                steps,
                                desired_steps,
                            );
                            results.push((setup.run(), setup));

                            test_count += 1;
                        }
                    }
                }
            }
        }
        eprintln!("test_count: {}", test_count);

        write_to_csv("data/generate_and_run_setups_0.csv", results)?;

        Ok(())
    }

    pub struct TestSetup {
        pub dt: f64,
        pub r0: f64,
        pub r1: f64,
        pub rho: f64,
        pub init_impact_v: f64,
        pub sep_dis: f64,
        pub steps: usize,
        pub desired_steps: usize,
    }

    impl TestSetup {
        pub fn new(
            dt: f64,
            r0: f64,
            r1: f64,
            rho: f64,
            init_impact_v: f64,
            sep_dis: f64,
            steps: usize,
            desired_steps: usize,
        ) -> Self {
            Self {
                dt,
                r0,
                r1,
                rho,
                init_impact_v,
                sep_dis,
                steps,
                desired_steps,
            }
        }

        /// Assuming a pair of particles. Simulation stops when the particles finish colliding when enabling the early_quit feature.
        pub fn run(&self) -> Result<TestResult, anyhow::Error> {
            use anyhow::{anyhow, ensure};

            let mut sys = KDTreeSystem::new(
                particle::two_unequal_bodies(
                    self.r0,
                    self.r1,
                    self.rho,
                    self.init_impact_v,
                    self.sep_dis,
                ),
                self.dt,
                self.desired_steps,
            );

            let (pre1, pre2) = {
                let pop = sys.pop.borrow();
                (pop[0], pop[1])
            };

            let pre_momentum = momentum(&sys.pop.borrow());

            #[cfg(feature = "early_quit")]
            let mut isColliding = false;

            #[cfg(feature = "early_quit")] //check_early_quit: &mut dyn FnMut(&[Particle]) -> bool
            let mut check_early_quit = |particles: &[Particle]| {
                let p0 = particles[0];
                let p1 = particles[1];

                let isCollidingRightNow = p0.is_colliding(&p1);

                match (isColliding, isCollidingRightNow) {
                    (false, true) => {
                        // starting to collide
                        isColliding = true;
                    }
                    (false, false) => {
                        // not colliding
                    }
                    (true, true) => {
                        // still colliding
                    }
                    (true, false) => {
                        // just stopped colliding
                        return true;
                    }
                }

                return false;
            };

            sys.run_with_quit_option(
                self.steps,
                #[cfg(feature = "early_quit")]
                &mut check_early_quit,
            );

            let (
                collision_step_count_actual,
                (real_impact_vel, _bad_exit_vel, max_penetration_depth),
            ) = {
                let soft_sphere_force = sys.pq.borrow();
                let impact_vel_tracker = soft_sphere_force.impact_vel.borrow();

                (
                    impact_vel_tracker
                        .get_updated_count(ParticleIndex(0), ParticleIndex(1))
                        .unwrap_or(0),
                    impact_vel_tracker
                        .get_analysis(ParticleIndex(0), ParticleIndex(1))
                        .ok_or(anyhow!("no collision found"))?,
                )
            };

            let (post1, post2) = {
                let pop = sys.pop.borrow();
                (pop[0], pop[1])
            };

            #[cfg(not(feature = "early_quit"))]
            let exit_vel = 0.;
            #[cfg(feature = "early_quit")]
            let exit_vel = post1.relative_speed(&post2); // if quitting right after the collision, the current speed is the exit speed

            let coeff_of_res = exit_vel / real_impact_vel;

            ensure!(
                max_penetration_depth > 0.,
                "min_center_separation_distance is somehow negative. This should never happen."
            );

            // divide by 2? the deformation would only be half
            let max_pen_depth_percent_1 = (max_penetration_depth / 2.) / pre1.r * 100.;
            let max_pen_depth_percent_2 = (max_penetration_depth / 2.) / pre2.r * 100.;

            ensure!(
                coeff_of_res > 0.,
                format!("Coefficient of restitution is negative: {}", coeff_of_res)
            );
            ensure!(
                coeff_of_res < 1.,
                format!(
                    "Coefficient of restitution is greater than 1: {}",
                    coeff_of_res
                )
            );

            let post_momentum = momentum(&sys.pop.borrow());

            ensure!(
                (post_momentum - pre_momentum).mag()
                    <= (f64::max(post_momentum.mag(), pre_momentum.mag())) * 0.01,
                format!(
                    "Momentum not conserved: {:e}",
                    (post_momentum - pre_momentum).mag()
                )
            );

            if pre1.p[0] < pre2.p[0] {
                ensure!(
                    post1.p[0] < post2.p[0],
                    "particles passed through each other"
                );
            } else {
                ensure!(
                    post1.p[0] > post2.p[0],
                    "particles passed through each other"
                );
            }

            ensure!(
                post1.p[2] == 0. && post1.p[1] == 0.,
                "particle 1 not on x axis"
            );

            ensure!(
                post2.p[2] == 0. && post2.p[1] == 0.,
                "particle 2 not on x axis"
            );

            Ok(TestResult {
                coeff_of_res,
                collision_step_count_actual,
                max_pen_depth_percent_1,
                max_pen_depth_percent_2,
                real_impact_vel,
            })
        }
    }

    pub struct TestResult {
        pub coeff_of_res: f64,
        pub collision_step_count_actual: usize,
        pub max_pen_depth_percent_1: f64,
        pub max_pen_depth_percent_2: f64,
        pub real_impact_vel: f64,
    }
}
