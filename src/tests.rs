use crate::fourthOrderInt::run_test;
use crate::test_automation::*;

/*
 * 0.03 does not converse energy when drag is 0
 */

const TIME_STEPS: [f64; 5] = [0.01, 0.006, 0.003, 0.001, 0.0003]; 
const SIGMOID_WIDTH_MODIFIER: [f64; 3] = [0.1, 0.03, 0.01];
const VELOCITIES: [f64; 5] = [3e-8, 1e-7, 3e-7, 1e-6, 3e-6];

pub fn test_suite_varying_time_steps() {
    let mut out = CSVOutput::new("data/test_suite_varying_time_steps.csv");
    out.writeHeader();

    let mut total = 0;
    let mut fails = 0;

    for dt in TIME_STEPS.iter() {
        let test = TestSetup::new(1e-7, *dt, 1e-7, 1e-7, 0.1, false);
        let result = run_test(&test, false);
        //result.print();

        match result {
            Ok(data) => out.writeEntry(&test, &data),
            Err(why) => {
                println!("{} for dt={:e}", why, dt);
                out.writeEntryFailed(&test);
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
                let test = TestSetup::new(*v_impact, *dt, 1e-7, 1e-7, *w, false);
                let result = run_test(&test, false);
                //result.print();
                
                match result {
                    Ok(data) => out.writeEntry(&test, &data),
                    Err(why) => {
                        println!("{} for dt={:e}, w={:e}, v_impact={:e}", why, dt, w, v_impact);
                        out.writeEntryFailed(&test);
                        fails += 1;
                    }
                }
                total += 1;
            }
        }
    }

    println!("Completed {}/{} tests successfully", total - fails, total);
}