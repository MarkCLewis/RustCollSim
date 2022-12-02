/*
 *
 *  d2x/dt2 + beta * dx/dt + omega'^2 * x = 0
 *
 *  beta = b/m
 *
 *  omega'^2 = k/m
 */

// for r, v_0 = 1e-7, rho = 0.88
// b = 3.4194745456729856e-20, k = 6.595530918688126e-18
//pub const B: f64 = 3.4194745456729856e-20;
//pub const K: f64 = 6.595530918688126e-18;

pub const PEN_RATIO_DEFAULT: f64 = 0.02;
pub const COEFF_RES: f64 = 0.5;

pub fn omega_0_from_k(k: f64, m: f64) -> f64 {
    (k / m).sqrt()
}

#[allow(dead_code)]
pub mod rotter {
    use std::f64::consts::PI;

    fn beta2(v_0: f64, pen_depth: f64) -> f64 {
        (-v_0 * 2. * crate::no_explode::COEFF_RES.ln() * crate::no_explode::COEFF_RES.sqrt())
            / (pen_depth * PI)
    }

    fn omega_0_sq(beta_val: f64) -> f64 {
        let ln_coeff_res_sq = crate::no_explode::COEFF_RES.ln() * crate::no_explode::COEFF_RES.ln();
        (beta_val * beta_val * (ln_coeff_res_sq + PI * PI)) / (4. * ln_coeff_res_sq)
    }

    // returns (b, k)
    fn b_and_k2(v_0: f64, m: f64, pen_depth: f64) -> (f64, f64) {
        let beta_val = beta2(v_0, pen_depth);
        let omega_0_sq_val = omega_0_sq(beta_val);

        (beta_val * m, omega_0_sq_val * m)
    }

    // returns (b, k)
    pub fn b_and_k(v_0: f64, m: f64, radius: f64) -> (f64, f64) {
        b_and_k2(v_0, m, radius * crate::no_explode::PEN_RATIO_DEFAULT)
    }
}

#[allow(dead_code)]
pub mod lewis {
    use std::f64::consts::PI;

    fn k(m: f64, v_i: f64, r: f64) -> f64 {
        let delta_r = crate::no_explode::PEN_RATIO_DEFAULT * r;
        m * v_i * v_i / (delta_r * delta_r)
    }

    fn c(k: f64, m: f64) -> f64 {
        2. * (k * m).sqrt() * crate::no_explode::COEFF_RES.ln() / PI
    }

    pub fn b_and_k(v_0: f64, m: f64, radius: f64) -> (f64, f64) {
        let k = k(m, v_0, radius);
        let c = c(k, m).abs();
        (c, k)
    }
}

#[allow(dead_code)]
pub mod schwartz {
    use std::f64::consts::PI;

    const CONST_OF_PROP: f64 = 1.; // TODO: what is this?
                                   // I think its about 1 as it works for the example given
    const MAX_PEN_RATIO: f64 = 0.02;

    fn k(m: f64, v_max: f64, x_max: f64) -> f64 {
        let tmp = v_max / x_max;
        m * CONST_OF_PROP * tmp * tmp
    }

    fn c(k: f64, m: f64) -> f64 {
        let lne = crate::no_explode::COEFF_RES.ln();
        -2. * (k * m / (PI * PI + lne * lne)).sqrt() * lne
    }

    pub fn b_and_k(v_max: f64, m: f64, r: f64) -> (f64, f64) {
        let k = k(m, v_max, r * MAX_PEN_RATIO).abs();
        let c = c(k, m).abs();
        (c, k)
    }
}
