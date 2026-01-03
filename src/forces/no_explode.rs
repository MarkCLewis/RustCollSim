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

// pub const PEN_RATIO_DEFAULT: f64 = 0.02;
// pub const COEFF_RES: f64 = 0.5;

use std::f64::consts::PI;

pub fn omega_0_from_k(k: f64, m: f64) -> f64 {
    (k / m).sqrt()
}

pub fn omega_l(k: f64, m: f64, b: f64) -> f64 {
    let omega_0 = omega_0_from_k(k, m);
    let omega_0_sq = omega_0 * omega_0;
    let b_sq = b * b;
    let omega_l_sq = omega_0_sq - b_sq / (4. * m * m);
    omega_l_sq.sqrt()
}

pub trait SpringDerivation {
    // pen fraction is the fraction of acceptable overlap, 0.02 is 2% overlap
    fn b_and_k(&self, v_0: f64, m: f64, radius: f64) -> (f64, f64);
    fn get_pen_fraction(&self) -> f64;
}

// Rotter derivation
pub struct Rotter {
    coeff_res: f64,
    pen_fraction: f64,
}

impl Rotter {
    #[inline(always)]
    fn beta2(&self, v_0: f64, pen_depth: f64) -> f64 {
        (-v_0 * 2. * self.coeff_res.ln() * self.coeff_res.sqrt()) / (pen_depth * PI)
    }

    #[inline(always)]
    fn omega_0_sq(&self, beta_val: f64) -> f64 {
        let ln_coeff_res_sq = self.coeff_res.ln() * self.coeff_res.ln();
        (beta_val * beta_val * (ln_coeff_res_sq + PI * PI)) / (4. * ln_coeff_res_sq)
    }

    #[inline(always)]
    // returns (b, k)
    fn b_and_k2(&self, v_0: f64, m: f64, pen_depth: f64) -> (f64, f64) {
        let beta_val = self.beta2(v_0, pen_depth);
        let omega_0_sq_val = self.omega_0_sq(beta_val);

        (beta_val * m, omega_0_sq_val * m)
    }
    pub fn new(coeff_res: f64, pen_fraction: f64) -> Self {
        Self {
            coeff_res,
            pen_fraction,
        }
    }
}

impl SpringDerivation for Rotter {
    fn b_and_k(&self, v_0: f64, m: f64, radius: f64) -> (f64, f64) {
        self.b_and_k2(v_0, m, radius * self.pen_fraction)
    }

    fn get_pen_fraction(&self) -> f64 {
        self.pen_fraction
    }
}

impl Default for Rotter {
    fn default() -> Self {
        Self::new(0.5, 0.02)
    }
}

pub struct Lewis {
    coeff_res: f64,
    pen_fraction: f64,
}

impl Lewis {
    #[inline(always)]
    fn k(&self, m: f64, v_i: f64, r: f64) -> f64 {
        let delta_r = self.pen_fraction * r;
        m * v_i * v_i / (delta_r * delta_r)
    }

    #[inline(always)]
    fn c(&self, k: f64, m: f64) -> f64 {
        2. * (k * m).sqrt() * self.coeff_res.ln() / PI
    }

    pub fn new(coeff_res: f64, pen_fraction: f64) -> Self {
        Self {
            coeff_res,
            pen_fraction,
        }
    }
}

impl SpringDerivation for Lewis {
    fn b_and_k(&self, v_0: f64, m: f64, radius: f64) -> (f64, f64) {
        let k = self.k(m, v_0, radius);
        let c = self.c(k, m).abs();
        (c, k)
    }

    fn get_pen_fraction(&self) -> f64 {
        self.pen_fraction
    }
}

impl Default for Lewis {
    fn default() -> Self {
        Self::new(0.5, 0.02)
    }
}

pub struct Schwartz {
    const_of_prop: f64, // TODO: what is this?
    // I think const_of_prop is about 1 as it works for the example given
    max_pen_ratio: f64,
    coeff_res: f64,
}

impl Schwartz {
    pub fn new(max_pen_ratio: f64, coeff_res: f64) -> Self {
        Self {
            const_of_prop: 1.,
            max_pen_ratio,
            coeff_res,
        }
    }

    #[inline(always)]
    fn k(&self, m: f64, v_max: f64, x_max: f64) -> f64 {
        let tmp = v_max / x_max;
        m * self.const_of_prop * tmp * tmp
    }

    #[inline(always)]
    fn c(&self, k: f64, m: f64) -> f64 {
        let lne = self.coeff_res.ln();
        -2. * (k * m / (PI * PI + lne * lne)).sqrt() * lne
    }
}

impl SpringDerivation for Schwartz {
    fn b_and_k(&self, v_max: f64, m: f64, r: f64) -> (f64, f64) {
        let k = self.k(m, v_max, r * self.max_pen_ratio).abs();
        let c = self.c(k, m).abs();
        (c, k)
    }

    fn get_pen_fraction(&self) -> f64 {
        self.max_pen_ratio
    }
}

impl Default for Schwartz {
    fn default() -> Self {
        Self::new(0.5, 0.02)
    }
}
