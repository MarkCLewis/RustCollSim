#[cfg(test)]
pub mod test_setup {
    use std::path::Path;

    use anyhow::ensure;

    use crate::{
        debugln,
        particle::{self, momentum, ParticleIndex},
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
            "initial_separation_distance",
            "simulation_max_step_count",
            "desired_coeff_of_res",
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
                        setup.sep_dis.to_string(),
                        setup.steps.to_string(),
                        setup.desired_coeff_of_res.to_string(),
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
                        setup.sep_dis.to_string(),
                        setup.steps.to_string(),
                        setup.desired_coeff_of_res.to_string(),
                    ])?;
                }
            }
        }

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
        pub desired_coeff_of_res: f64,
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
            desired_coeff_of_res: f64,
        ) -> Self {
            Self {
                dt,
                r0,
                r1,
                rho,
                init_impact_v,
                sep_dis,
                steps: usize::max(1, steps), // we need at least one step for things to happen
                desired_steps: usize::max(1, desired_steps),
                desired_coeff_of_res,
            }
        }

        /// Assuming a pair of particles. Simulation stops when the particles finish colliding when enabling the early_quit feature.
        pub fn run(&self) -> Result<TestResult, anyhow::Error> {
            use anyhow::anyhow;

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
                self.desired_coeff_of_res,
                None,
            );

            debugln!("SETUP r0={}, r1={}, rho={}, init_impact_v={}, sep_dis={}, dt={}, steps={}, desired_steps={}", self.r0, self.r1, self.rho, self.init_impact_v, self.sep_dis, self.dt, self.steps, self.desired_steps);

            let (pre1, pre2) = {
                let pop = sys.pop.borrow();
                (pop[0], pop[1])
            };

            let pre_momentum = momentum(&sys.pop.borrow());

            #[cfg(feature = "early_quit")]
            let mut isColliding = false;

            #[cfg(feature = "early_quit")] //check_early_quit: &mut dyn FnMut(&[Particle]) -> bool
            let mut check_early_quit = |particles: &[particle::Particle]| {
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

            if pre1.p.x() < pre2.p.x() {
                ensure!(
                    post1.p.x() < post2.p.x(),
                    "particles passed through each other"
                );
            } else {
                ensure!(
                    post1.p.x() > post2.p.x(),
                    "particles passed through each other"
                );
            }

            ensure!(!post1.is_colliding(&post2), "particles are still colliding");

            ensure!(
                post1.p.z() == 0. && post1.p.y() == 0.,
                "particle 1 not on x axis"
            );

            ensure!(
                post2.p.z() == 0. && post2.p.y() == 0.,
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

    impl std::fmt::Display for TestResult {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            writeln!(f, "TestResult")?;
            writeln!(f, "Coefficient of Restitution: {:.3}", self.coeff_of_res)?;
            writeln!(
                f,
                "Actual collision step count: {}",
                self.collision_step_count_actual
            )?;
            writeln!(
                f,
                "Max penetration depth percent 1: {:.3}",
                self.max_pen_depth_percent_1
            )?;
            writeln!(
                f,
                "Max penetration depth percent 2: {:.3}",
                self.max_pen_depth_percent_2
            )?;
            writeln!(f, "Real impact velocity: {:.3e}", self.real_impact_vel)
        }
    }
}
