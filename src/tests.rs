use crate::fourthOrderInt::run_test;
use crate::test_automation::*;

/*
 * 0.03 does not converse energy when drag is 0
 */
const DUMMY: [f64; 1] = [f64::NAN];


const TIME_STEPS: [f64; 5] = [0.006, 0.003, 0.001, 0.0003, 0.0001]; 
const SIGMOID_WIDTH_MODIFIER: [f64; 3] = [0.1, 0.03, 0.01];
const VELOCITIES: [f64; 5] = [3e-8, 1e-7, 3e-7, 1e-6, 3e-6];
const RADII: [(f64, f64); 6] = [(1e-7, 1e-7), (3e-8, 3e-8), (1e-8, 1e-8), (3e-8, 1e-8), (1e-7, 3e-8), (1e-7, 1e-8)];


const K_B_DERIVATIONS: [KBCalculator; 3] = [KBCalculator::ROTTER, KBCalculator::LEWIS, KBCalculator::SCHWARTZ];
const INTEGRATORS: [Integrator; 2] = [Integrator::Jerk, Integrator::KickStepKick];

const BLEND_FUNC: [BlendFunc; 2] = [BlendFunc::SIGMOID, BlendFunc::STEP];

// Remember to consider different approximations for the impact velocity
// Tests with and without sigmoids

pub fn test_suite_full() {
    let mut out = CSVOutput::new("data/test_suite_full.csv");
    out.writeHeader();

    let mut total = 0;
    let mut fails = 0;

    let mut sigmoid_fails = 0;
    let mut step_fails = 0;

    for blend in BLEND_FUNC.iter() {
        for int in INTEGRATORS.iter() {
            for k_b_deriv in K_B_DERIVATIONS.iter() {
                for (r0, r1) in RADII.iter() {
                    for dt in TIME_STEPS.iter() {
                        
                        /* match int {
                            Integrator::Jerk => SIGMOID_WIDTH_MODIFIER.iter(),
                            Integrator::KickStepKick => DUMMY.iter() // only jerk uses sigmoids
                        } */
                        for w in SIGMOID_WIDTH_MODIFIER.iter() {
                            for v_impact in VELOCITIES.iter() {
                                if v_impact * dt > 0.1 * (r0 + r1) {
                                    continue;
                                }
                                //assert_eq!(v_impact * dt > (r0 + r1), false);
                                let test = TestSetup::new(*v_impact, *dt, *r0, *r1, *w, false, *k_b_deriv, *int, *blend);
                                let result = run_test(&test, false);
                                
                                //result.print();
                                
                                match result {
                                    Ok(data) => out.writeEntry(&test, &data.0, &data.1),
                                    Err((why, data)) => {
                                        println!("{} for dt={:e}, w={:e}, v_impact={:e}", why, dt, w, v_impact);
                                        out.writeEntryFailed(&test, &data);
                                        fails += 1;
                                        match blend {
                                            BlendFunc::SIGMOID => {
                                                sigmoid_fails += 1
                                            },
                                            BlendFunc::STEP => {
                                                step_fails += 1
                                            }
                                        };
                                    }
                                }
                                total += 1;
                            }
                        }
                    }
                }
            }
        }
    }


    println!("Completed {}/{} tests successfully", total - fails, total);

    println!("Sigmoid completed {}/{} tests successfully", total/2 - sigmoid_fails, total/2);
    println!("Step completed {}/{} tests successfully", total/2 - step_fails, total/2);
}

pub fn test_suite_varying_time_steps() {
    let mut out = CSVOutput::new("data/test_suite_varying_time_steps.csv");
    out.writeHeader();

    let mut total = 0;
    let mut fails = 0;

    for dt in TIME_STEPS.iter() {
        if 1e-7 * dt > 0.1 * (2e-7) {
            continue;
        }

        let test = TestSetup::new(1e-7, *dt, 1e-7, 1e-7, 0.1, false, KBCalculator::ROTTER, Integrator::Jerk, BlendFunc::SIGMOID);
        let result = run_test(&test, false);
        //result.print();

        match result {
            Ok(data) => out.writeEntry(&test, &data.0, &data.1),
            Err((why, data)) => {
                println!("{} for dt={:e}", why, dt);
                out.writeEntryFailed(&test, &data);
                fails += 1;
            }
        }
        total += 1;
    }
    println!("Completed {}/{} tests successfully", total - fails, total);
}

pub fn test_suite_1() {
    let mut out = CSVOutput::new("data/test_suite_1.csv");
    out.writeHeader();

    let mut total = 0;
    let mut fails = 0;

    for dt in TIME_STEPS.iter() {
        for w in SIGMOID_WIDTH_MODIFIER.iter() {
            for v_impact in VELOCITIES.iter() {
                if v_impact * dt > 0.1 * (2e-7) {
                    continue;
                }
                let test = TestSetup::new(*v_impact, *dt, 1e-7, 1e-7, *w, false, KBCalculator::ROTTER, Integrator::Jerk, BlendFunc::SIGMOID);
                let result = run_test(&test, false);
                //result.print();
                
                match result {
                    Ok(data) => out.writeEntry(&test, &data.0, &data.1),
                    Err((why, data)) => {
                        println!("{} for dt={:e}, w={:e}, v_impact={:e}", why, dt, w, v_impact);
                        out.writeEntryFailed(&test, &data);
                        fails += 1;
                    }
                }
                total += 1;
            }
        }
    }

    println!("Completed {}/{} tests successfully", total - fails, total);
}