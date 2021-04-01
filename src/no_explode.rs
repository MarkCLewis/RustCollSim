

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
const COEFF_RES: f64 = 0.5;

pub mod compute {
    use std::f64::consts::PI;

    //pub const R_DEFAULT: f64 = 1e-7; // v_o 1e-7
    
    //const PEN_MAX_DEFAULT: f64 = R_DEFAULT * PEN_RATIO_DEFAULT;
    
    const ROOT_COEFF_RES: f64 = 0.7071067811865476;
    const LN_COEFF_RES: f64 = -0.6931471805599453; //COEFF_RES.ln(); // ln(c)
    const ROOT_4_COEFF_RES: f64 = 0.8408964152537145; //COEFF_RES.sqrt().sqrt(); // c^{1/4}
    const LN_COEFF_RES_SQ: f64 = LN_COEFF_RES * LN_COEFF_RES;

    pub fn beta2(v_0: f64, pen_depth: f64) -> f64 {
        ( - v_0 * 2. * LN_COEFF_RES * ROOT_COEFF_RES ) / ( pen_depth * PI )
    }

    // pub fn beta(v_0: f64) -> f64 {
    //     beta2(v_0, PEN_MAX_DEFAULT)
    // }
    
    pub fn omega_0_sq(beta_val: f64) -> f64 {
        ( beta_val * beta_val * (LN_COEFF_RES_SQ + PI * PI) ) / ( 4. * LN_COEFF_RES_SQ )
    }

    // returns (b, k)
    pub fn b_and_k2(v_0: f64, m: f64, pen_depth: f64) -> (f64, f64) {
        let beta_val = beta2(v_0, pen_depth);
        let omega_0_sq_val = omega_0_sq(beta_val);

        (beta_val * m, omega_0_sq_val * m)
    }

    // returns (b, k)
    pub fn b_and_k3(v_0: f64, m: f64, radius: f64) -> (f64, f64) {
        b_and_k2(v_0, m, radius * crate::no_explode::PEN_RATIO_DEFAULT)
    }

    /*
     * returns (b, k)
     */
    // pub fn b_and_k(v_0: f64, m: f64) -> (f64, f64) {
    //     let beta_val = beta(v_0);
    //     let omega_0_sq_val = omega_0_sq(beta_val);

    //     (beta_val * m, omega_0_sq_val * m)
    // }
}


pub mod lewis {
    use std::f64::consts::PI;

    pub fn k(m: f64, v_i: f64, r: f64) -> f64 {
        let delta_r = crate::no_explode::PEN_RATIO_DEFAULT * r;
        m * v_i * v_i / (delta_r * delta_r)
    }

    pub fn c(k: f64, m: f64) -> f64 {
        2. * (k * m).sqrt() * crate::no_explode::COEFF_RES.ln() / PI
    }

    pub fn b_and_k(v_0: f64, m: f64, radius: f64) -> (f64, f64) {
        let k = k(m, v_0, radius);
        let c = c(k, m).abs();
        (c, k)
    }
}