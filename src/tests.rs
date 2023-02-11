#[cfg(test)]
mod tests {
    use crate::particle::momentum;
    use crate::vectors::Vector;
    use crate::{no_explode::COEFF_RES, particle::ParticleIndex};
    use anyhow::{ensure, Context, Result};

    fn percent_error(actual: f64, expected: f64) -> f64 {
        ((actual - expected) / expected * 100.).abs()
    }

    fn assert_equal_orders_of_mag(
        actual: f64,
        expected: f64,
        allowed_difference: f64,
    ) -> Result<()> {
        let diff = (actual.log10() - expected.log10()).abs();

        ensure!(diff < allowed_difference);
        Ok(())
    }

    fn assert_coeff_of_res(
        v_before: [f64; 3],
        v_after: [f64; 3],
        percentage_error: f64,
    ) -> Result<()> {
        let coeff = Vector(v_after).mag() / Vector(v_before).mag();
        // assert!(coeff < COEFF)
        let error = COEFF_RES * percentage_error;
        ensure!(
            coeff <= COEFF_RES + error,
            "Coeff of res too small, got {} but expected {}. Error: {:.2}%",
            coeff,
            COEFF_RES,
            percent_error(coeff, COEFF_RES)
        );
        ensure!(
            coeff >= COEFF_RES - error,
            "Coeff of res too small, got {} but expected {}. Error: {:.2}%",
            coeff,
            COEFF_RES,
            percent_error(coeff, COEFF_RES)
        );

        Ok(())
    }

    fn pair_collision_run(
        dt: f64,
        r: f64,
        rho: f64,
        init_impact_v: f64,
        sep_dis: f64,
        steps: usize,
    ) -> Result<()> {
        use crate::{particle, system::KDTreeSystem};

        let collision_step_count = 15;

        let mut sys = KDTreeSystem::new(
            particle::two_equal_bodies(r, rho, init_impact_v, sep_dis),
            dt,
            collision_step_count,
        );

        let (pre1, pre2) = {
            let pop = sys.pop.borrow();
            (pop[0], pop[1])
        };

        let pre_momentum = momentum(&sys.pop.borrow());

        sys.run(steps);

        let collision_step_count_actual = sys
            .pq
            .borrow()
            .impact_vel
            .borrow()
            .get_updated_count(ParticleIndex(0), ParticleIndex(1))
            .unwrap_or(0);

        assert_equal_orders_of_mag(
            collision_step_count_actual as f64,
            collision_step_count as f64,
            0.2,
        )
        .with_context(|| {
            format!(
                "Got {} but expected {}",
                collision_step_count_actual, collision_step_count
            )
        })?;

        println!(
            "collision_step_count_actual: {}",
            collision_step_count_actual
        );

        let (post1, post2) = {
            let pop = sys.pop.borrow();
            (pop[0], pop[1])
        };

        let post_momentum = momentum(&sys.pop.borrow());

        assert_coeff_of_res(pre1.v, post1.v, 0.20)?;
        assert_coeff_of_res(pre2.v, post2.v, 0.20)?;

        ensure!(
            (post_momentum - pre_momentum).mag()
                <= (f64::max(post_momentum.mag(), pre_momentum.mag())) * 0.01,
            "Momentum changed from {} to {}",
            pre_momentum.mag(),
            post_momentum.mag()
        );

        if pre1.p[0] < pre2.p[1] {
            ensure!(
                post1.p[0] < post2.p[0],
                "particles passed through each other"
            );
        } else {
            ensure!(
                post1.p[0] >= post2.p[0],
                "particles passed through each other"
            );
        }

        ensure!(post1.p[2] == 0.);
        ensure!(post2.p[2] == 0.);

        Ok(())
    }

    #[test]
    /// to check gravity
    fn test_sun_planet() {
        use std::f64::consts::PI;

        use crate::{particle, system::KDTreeSystem};

        let dt = 1e-3;

        let mut sys = KDTreeSystem::new(particle::two_bodies(), dt, 10);

        let (sun, planet) = {
            let pop = sys.pop.borrow();
            (pop[0], pop[1])
        };

        sys.run((2. * PI / dt) as usize);

        let (sun2, planet2) = {
            let pop = sys.pop.borrow();
            (pop[0], pop[1])
        };

        let sun_drift = (Vector(sun.p) - Vector(sun2.p)).mag();
        let planet_drift = (Vector(planet.p) - Vector(planet2.p)).mag();
        let planet_v_drift = (Vector(planet.v) - Vector(planet2.v)).mag();

        assert!(sun_drift < 2e-4);
        assert!(planet_drift < 2e-4);
        assert!(Vector(sun2.v).mag() < 2e-4);
        assert!(planet_v_drift < 2e-4);
        assert_eq!(sun2.p[2], 0.);
        assert_eq!(planet2.p[2], 0.);
    }

    #[test]
    /// to check collisions
    /// this test is invoked by analysis/STEP.py, its not meant to pass
    /// it is meant to collect debug information
    fn test_2_bodies() -> Result<()> {
        let r = 1e-7;
        let rho = 0.88;
        let init_impact_v = 2. * r;
        let sep_dis = 2.2 * r; // x = 1.1r
        let dt = 0.025; //1e-3;

        pair_collision_run(dt, r, rho, init_impact_v, sep_dis, (0.25 / dt) as usize)?;

        ensure!(false, "test is ok");

        Ok(())
    }

    #[test]
    fn test_2_bodies_variations_of_dt() -> Result<()> {
        let r = 1e-7;
        let rho = 0.88;
        let init_impact_v = 2. * r;
        let sep_dis = 2.2 * r; // x = 1.1r

        // total simulation time is 0.25

        for dt in vec![0.25, 0.05, 0.025, 0.005, 0.0025] {
            pair_collision_run(dt, r, rho, init_impact_v, sep_dis, (0.25 / dt) as usize)?;
        }

        Ok(())
    }

    // #[test]
    // /// to check collisions
    // fn test_2_bodies_speed() {
    //     let r = 1e-7;
    //     let rho = 0.88;
    //     let init_impact_v = 2. * r * 20.;
    //     let sep_dis = 2.2 * r; // x = 1.1r

    //     pair_collision_run(r, rho, init_impact_v, sep_dis, 11);
    // }
}
