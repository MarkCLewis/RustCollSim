use crate::fourthOrderInt::run_test;
use crate::test_automation::*;

const TIME_STEPS: [f64; 5] = [0.03, 0.01, 0.003, 0.001, 0.0003]; 
const SIGMOID_WIDTH_MODIFIER: [f64; 3] = [0.1, 0.03, 0.01];

pub fn test_suite_varying_time_steps() {
    let mut out = CSVOutput::new("data/test_suite_varying_time_steps.csv");
    out.writeHeader();


    for dt in TIME_STEPS.iter() {
        let test = TestSetup::new(1e-7, *dt, 1e-7, 1e-7, 0.1);
        let result = run_test(&test, false);
        //result.print();
        
        out.writeEntry(&test, &result);
    }
}

pub fn test_suite_1() {
    let mut out = CSVOutput::new("data/test_suite_1.csv");
    out.writeHeader();

    for dt in TIME_STEPS.iter() {
        for w in SIGMOID_WIDTH_MODIFIER.iter() {
            let test = TestSetup::new(1e-7, *dt, 1e-7, 1e-7, *w);
            let result = run_test(&test, false);
            //result.print();
            
            out.writeEntry(&test, &result);
        }
    }
}