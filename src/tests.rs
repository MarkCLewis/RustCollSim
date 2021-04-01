use crate::fourthOrderInt::run_test;
use crate::test_automation::*;

/*
 * 0.03 does not converse energy when drag is 0
 */

const TIME_STEPS: [f64; 5] = [0.01, 0.006, 0.003, 0.001, 0.0003]; 
const SIGMOID_WIDTH_MODIFIER: [f64; 3] = [0.1, 0.03, 0.01];
const VELOCITIES: [f64; 5] = [3e-8, 1e-7, 3e-7, 1e-6, 3e-6];
const RADII: [(f64, f64); 6] = [(1e-7, 1e-7), (3e-8, 3e-8), (1e-8, 1e-8), (1e-7, 1e-7), (1e-7, 3e-8), (1e-7, 1e-8)];

const K_B_DERIVATIONS: [KBCalculator; 3] = [KBCalculator::ROTTER, KBCalculator::LEWIS, KBCalculator::SCHWARTZ];
const INTEGRATORS: [Integrator; 2] = [Integrator::Jerk, Integrator::KickStepKick];

// Remember to consider different approximations for the impact velocity
// Tests with and without sigmoids

pub fn test_suite_full() {
    let mut out = CSVOutput::new("data/test_suite_full.csv");
    out.writeHeader();

    let mut total = 0;
    let mut fails = 0;

    for int in INTEGRATORS.iter() {
        for k_b_deriv in K_B_DERIVATIONS.iter() {
            for (r0, r1) in RADII.iter() {
                for dt in TIME_STEPS.iter() {
                    for w in SIGMOID_WIDTH_MODIFIER.iter() {
                        for v_impact in VELOCITIES.iter() {
                            let test = TestSetup::new(*v_impact, *dt, *r0, *r1, *w, false, *k_b_deriv, *int);
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
            }
        }
    }


    println!("Completed {}/{} tests successfully", total - fails, total);
}

pub fn test_suite_varying_time_steps() {
    let mut out = CSVOutput::new("data/test_suite_varying_time_steps.csv");
    out.writeHeader();

    let mut total = 0;
    let mut fails = 0;

    for dt in TIME_STEPS.iter() {
        let test = TestSetup::new(1e-7, *dt, 1e-7, 1e-7, 0.1, false, KBCalculator::ROTTER, Integrator::Jerk);
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
                let test = TestSetup::new(*v_impact, *dt, 1e-7, 1e-7, *w, false, KBCalculator::ROTTER, Integrator::Jerk);
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