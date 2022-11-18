use crate::{
    particle::{energy, momentum},
    vectors::Vector,
};

fn percent_error(actual: f64, expected: f64) -> f64 {
    (actual - expected) / expected * 100.
}

fn pair_collision_run(r: f64, rho: f64, init_impact_v: f64, sep_dis: f64, steps: usize) {
    use crate::{particle, system::KDTreeSystem};

    let dt = 1e-3;

    let mut sys = KDTreeSystem::new(
        particle::two_equal_bodies(r, rho, init_impact_v, sep_dis),
        dt,
    );

    let (pre1, pre2) = {
        let pop = sys.pop.borrow();
        (pop[0], pop[1])
    };

    let pre_momentum = momentum(&sys.pop.borrow());
    let pre_energy = energy(&sys.pop.borrow());

    sys.run(steps);

    let (post1, post2) = {
        let pop = sys.pop.borrow();
        (pop[0], pop[1])
    };

    let post_momentum = momentum(&sys.pop.borrow());
    let post_energy = energy(&sys.pop.borrow());

    // let sun_drift = (Vector(sun.p) - Vector(sun2.p)).mag();
    // let planet_v_drift = (Vector(planet.v) - Vector(planet2.v)).mag();

    assert!((Vector(pre1.p) - Vector(post1.p)).mag() < r * 1e-3);
    assert!((Vector(pre2.p) - Vector(post2.p)).mag() < r * 1e-3);

    // this is an elastic collision - prob it should be one

    // velocities should switch
    assert!((Vector(pre1.v) - Vector(post2.v)).mag() < r * 3e-3);
    assert!((Vector(pre2.v) - Vector(post1.v)).mag() < r * 3e-3);

    // velocities should flip
    assert!((Vector(pre1.v) + Vector(post1.v)).mag() < r * 3e-3);
    assert!((Vector(pre2.v) + Vector(post2.v)).mag() < r * 3e-3);

    // conservation of momentum
    assert_eq!((pre_momentum - post_momentum).mag(), 0.);

    let coeff_of_res1 = Vector(post1.v).mag() / Vector(pre1.v).mag();
    let coeff_of_res2 = Vector(post2.v).mag() / Vector(pre2.v).mag();

    // assert!(false, "{}", coeff_of_res1);

    // coeff_of_res is around 0.998

    // assert!(
    //     false,
    //     "{} {} {}",
    //     pre_energy,
    //     post_energy,
    //     pre_energy / post_energy
    // );

    assert_eq!(post1.p[2], 0.);
    assert_eq!(post2.p[2], 0.);
}

#[cfg(test)]
mod tests {
    use crate::{tests::pair_collision_run, vectors::Vector};

    #[test]
    /// to check gravity
    fn test_sun_planet() {
        use std::f64::consts::PI;

        use crate::{particle, system::KDTreeSystem};

        let dt = 1e-3;

        let mut sys = KDTreeSystem::new(particle::two_bodies(), dt);

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
    fn test_2_bodies() {
        let r = 1e-7;
        let rho = 0.88;
        let init_impact_v = 2. * r;
        let sep_dis = 2.2 * r; // x = 1.1r

        pair_collision_run(r, rho, init_impact_v, sep_dis, 250);
    }

    #[test]
    fn test_2_bodies_robust() {
        let rho = 0.88;
        for r in &[1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10] {
            let init_impact_v = 2. * r;
            let sep_dis = 2.2 * r; // x = 1.1r
            pair_collision_run(*r, rho, init_impact_v, sep_dis, 250);
        }
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
