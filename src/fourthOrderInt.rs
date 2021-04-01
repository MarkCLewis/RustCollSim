//use crate::data::basic::*;
use crate::vectors::Vector;
use crate::graphicsM;
use crate::graphics;

use crate::test_automation::*;
use std::f64::consts::PI;

pub fn scale(data: &mut Vec<Vector>, scalar: f64) {
    for x in data.iter_mut() {
        *x = *x * scalar;
    }
}

fn sigmoid(delta: f64) -> f64 {
    // sigmoid(100) = 1.0
    // sigmoid(-100) = 3.7200759760208356e-44 ~ 0
    if delta > 100. { 1. }
    else if delta < -100. { 0. }
    else {
        1.0 / (1.0 + (-delta).exp())
    }
}

fn sigmoidDot(deltaDot: f64, delta: f64) -> f64 {
    // sigmoidDot(deltaDot, -100) = deltaDot * 3.7200759760208356e-44
    if delta < -100. { 0. }
    else {
        let denom = 1. + (-delta).exp();
        deltaDot * (-delta).exp() / (denom * denom) 
    }
}

pub fn calcAccJerk(data: &mut TestData, test: &TestSetup) {

    for i in 0..(data.pos.len()) {
        data.acc[i] = Vector(0., 0., 0.);
        data.jerk[i] = Vector(0., 0., 0.);
    }

    for i in 0..(data.pos.len()) {
        for j in (i+1)..(data.pos.len()) {
            let rji = data.pos[j] - data.pos[i]; // relative pos
            let vji = data.vel[j] - data.vel[i]; // relative vel

            let rSq = rji.dot(&rji);
            let r = rSq.sqrt();
            let rCube = r * rSq;

            let rv_r2 = rji.dot(&vji) / rSq;

            let x_hat = rji / r;
            let delta = r - (data.rad[i] + data.rad[j]);

            let delta_dot = x_hat.dot(&vji);

            let x_hat_dot = (vji - x_hat * vji.dot(&x_hat)) / r;

            //println!("{} {} {} {} {} {}", rji.is_finite(), vji.is_finite(), r2, rv_r2, r, r3);

            let da = rji / rCube;
            let dj = (vji - rji * 3. * rv_r2) / rCube;
            let massi = 4. * PI * test.rho / 3. * data.rad[i] * data.rad[i] * data.rad[i];
            let massj = 4. * PI * test.rho / 3. * data.rad[j] * data.rad[j] * data.rad[j];

            let sig_pos_i = sigmoid(delta * data.sig_c[i]);
            let sig_pos_j = sigmoid(delta * data.sig_c[j]);

            let sig_neg_i = sigmoid(-delta * data.sig_c[i]);
            let sig_neg_j = sigmoid(-delta * data.sig_c[j]);

            let sig_dot_pos_i = sigmoidDot(delta_dot * data.sig_c[i], delta * data.sig_c[i]);
            let sig_dot_pos_j = sigmoidDot(delta_dot * data.sig_c[j], delta * data.sig_c[j]);

            let sig_dot_neg_i = sigmoidDot(-delta_dot * data.sig_c[i], -delta * data.sig_c[i]);
            let sig_dot_neg_j = sigmoidDot(-delta_dot * data.sig_c[j], -delta * data.sig_c[j]);

            //if delta > 0. {
                // no collision
            
            let ai_g = da * massj;
            let aj_g = da * massi;
            
            let ji_g = dj * massj;
            let jj_g = dj * massi;
            
            // gravity
            data.acc[i] += ai_g * sig_pos_i;
            data.acc[j] -= aj_g * sig_pos_j;

            data.jerk[i] += ji_g * sig_dot_pos_i + ai_g * sig_pos_i; // TODO: ????
            data.jerk[j] -= jj_g * sig_dot_pos_j + ai_g * sig_pos_j;
            // mj = sig dot * force + sig * jerk

            //}
            //else {//if delta < 0. {
                // collision!
            
            // collision
            let f_spring = x_hat * -test.k * delta;
            let f_damp = vji * -test.b;

            let f_total = f_spring + f_damp;

            // F = m_i * a_i
            // a_i = F / m_i
            let ai = f_total / massi;
            let aj = f_total / massj;
            let aji = aj - ai;

            // someone somewhere suggested this as d/dt F
            let yank_spring = (x_hat * delta_dot + x_hat_dot * delta) * -test.k;
            let yank_damp = aji * -test.b;

            let yank_total = yank_spring + yank_damp;

            // F = m_i * a_i
            // d/dt F = m_i * jerk_i
            // jerk_i = d/dt F / m_i

            let ji = yank_total / massi;
            let jj = yank_total / massj;

            data.acc[i] -= ai * sig_neg_i;
            data.acc[j] += aj * sig_neg_j;

            // jerk[i] -= ji; // TODO: ???
            // jerk[j] += jj;
            // mj = sig dot * force + sig * jerk
            data.jerk[i] -= ai * sig_dot_neg_i + ji * sig_neg_i;
            data.jerk[j] += aj * sig_dot_neg_j + jj * sig_neg_j;
            // }
        }
    }
}

fn predictStep(pos: &mut Vec<Vector>, vel: &mut Vec<Vector>, acc: &Vec<Vector>, jerk: &Vec<Vector>, dt: f64) {
    let dt2 = dt*dt/2.0;
    let dt3 = dt2*dt/6.0;

    // if !dt2.is_finite() || !dt3.is_finite() {
    //     panic!("aaaaaaaaaaaaaaaaaa");
    // }

    for i in 0..pos.len() {
        // TODO: These might not be optimally efficient without expression templates.
        // TODO: figure out what this todo means
        //println!("{}, {}, {}, {}, {}, {}", vel[i].is_finite(), dt, acc[i].is_finite(), dt2, jerk[i].is_finite(), dt3);
        pos[i] += vel[i] * dt + acc[i] * dt2 + jerk[i] * dt3;
        vel[i] += acc[i] * dt + jerk[i] * dt2;
    }
}

fn correctStep(pos: &mut Vec<Vector>, vel: &mut Vec<Vector>, acc: &Vec<Vector>, jerk: &Vec<Vector>, 
    oldPos: &Vec<Vector>, oldVel: &Vec<Vector>, oldAcc: &Vec<Vector>, oldJerk: &Vec<Vector>, dt: f64) {
    // TODO: These might not be optimally efficient without expression templates.
    // TODO: figure out what this todo means
    for i in 0..pos.len() {
        vel[i] = oldVel[i] + (oldAcc[i] + acc[i])*dt/2. + (oldJerk[i] - jerk[i])*dt*dt/12.;
        pos[i] = oldPos[i] + (oldVel[i] + vel[i])*dt/2. + (oldAcc[i] - acc[i])*dt*dt/12.;
    }
}

fn kinetic_energy(vel: &Vec<Vector>, rad: &Vec<f64>, rho: f64) -> Vec<f64> {
    let mut vec: Vec<f64> = Vec::new();
    for i in 0..vel.len() {
        let mass = 4./3. * PI * rad[i] * rad[i] * rad[i] * rho;
        vec.push(0.5 * mass * vel[i].magSq());
    }
    return vec;
}

fn potential_energy2(pos: &Vec<Vector>, rad: &Vec<f64>, rho: f64) -> Vec<f64> {
    // -G (mM) / R

    let mut vec: Vec<f64> = Vec::new();
    for i in 0..pos.len()
    {
        let mut energy = 0.;
        for j in 0..pos.len()
        {
            if i == j {
                continue;
            }

            // compute R
            let r = (pos[i] - pos[j]).mag();
            let massi = 4./3. * PI * rad[i] * rad[i] * rad[i] * rho;
            let massj = 4./3. * PI * rad[j] * rad[j] * rad[j] * rho;
            // G is 1
            //energy += - (masses[i / 6] * masses[j / 6] / r);
            energy += -massi * massj / r;
            //printf("PE[%d] on [%d] = %e\n", i / 6, j / 6, - (masses[i / 6] * masses[j / 6] / r));
        }
        vec.push(energy);
    }
    //printf("U = %e\n", energy);
    return vec;
}

pub fn evolveStep(data: &mut TestData, test: &TestSetup) {
    // TODO: This isn't ideally efficient. Better to keep two vectors around and reuse, but it will do for now.
    let oldPos = data.pos.clone();
    let oldVel = data.vel.clone();
    let oldAcc = data.acc.clone();
    let oldJerk = data.jerk.clone();

    predictStep(&mut data.pos, &mut data.vel, &data.acc, &data.jerk, test.dt);
    //assert_eq!(pos[0].is_finite(), true);
    calcAccJerk(data, test);
    //assert_eq!(pos[0].is_finite(), true);
    correctStep(&mut data.pos, &mut data.vel, &data.acc, &data.jerk, &oldPos, &oldVel, &oldAcc, &oldJerk, test.dt);
    //assert_eq!(pos[0].is_finite(), true);
}

fn integrate_kick_step_kick_1(pos: &mut Vec<Vector>, vel: &mut Vec<Vector>, acc: &Vec<Vector>, dt: f64) {
    for i in 0..pos.len() {
        // TODO: These might not be optimally efficient without expression templates.
        // TODO: figure out what this todo means
        //println!("{}, {}, {}, {}, {}, {}", vel[i].is_finite(), dt, acc[i].is_finite(), dt2, jerk[i].is_finite(), dt3);
        vel[i] += acc[i] * dt/2.;
        pos[i] += vel[i] * dt;
    }
}

fn integrate_kick_step_kick_2(vel: &mut Vec<Vector>, acc: &Vec<Vector>, dt: f64) {
    for i in 0..vel.len() {
        // TODO: These might not be optimally efficient without expression templates.
        // TODO: figure out what this todo means
        //println!("{}, {}, {}, {}, {}, {}", vel[i].is_finite(), dt, acc[i].is_finite(), dt2, jerk[i].is_finite(), dt3);
        vel[i] += acc[i] * dt/2.;
    }
}

pub fn evolveStepKickStepKick(data: &mut TestData, test: &TestSetup) {
    // TODO: This isn't ideally efficient. Better to keep two vectors around and reuse, but it will do for now.

    // this assumes acc has already been calculated

    integrate_kick_step_kick_1(&mut data.pos, &mut data.vel, &data.acc, test.dt);

    calcAccJerk(data, test);

    integrate_kick_step_kick_2(&mut data.vel, &data.acc, test.dt);
    //assert_eq!(pos[0].is_finite(), true);

    //assert_eq!(pos[0].is_finite(), true);
}

// const n: f64 = 1.0;
// const kappa: f64 = 1.0;
// const n_z: f64 = 1.0;

// // Call this after all others so the accels are completed by the time this is called.
// fn hillsForce(pos: &Vector, vel: &Vector, acc: &mut Vector, jerk: &mut Vector) {
//     acc.0 += 2.0 * n * vel.1 - (kappa * kappa - 4.0 * n * n) * pos.0;
//     acc.1 += -2.0 * n * vel.0;
//     acc.2 += -n_z * n_z * pos.2;
//     jerk.0 += 2.0 * n * acc.1 - (kappa * kappa - 4.0 * n * n) * vel.0;
//     jerk.1 += -2.0 * n * acc.0;
//     jerk.2 += -n_z * n_z * vel.2;
// }


// pub fn main() {
//     // gravity test

//     let dt = 0.001 * 2. * PI;

//     let mut pos: Vec<Vector> = vec!(Vector(0.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0));
//     let mut vel: Vec<Vector> = vec!(Vector(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0));
//     let rad: Vec<f64> = vec!(1e-7, 1e-14);
//     let rho: f64 = 3.0 / (4.0 * PI *rad[0]*rad[0]*rad[0]); // what?
//     let mut acc: Vec<Vector> = vec!(Vector(0., 0., 0.), Vector(0., 0., 0.));
//     let mut jerk: Vec<Vector> = vec!(Vector(0., 0., 0.), Vector(0., 0., 0.));

//     pos[1].print();
//     vel[1].print();

//     let mut g = graphicsM!(1e-0);

//     calcAccJerk(&pos, &vel, &rad, rho, &mut acc, &mut jerk);
//     let mut t = 0.;
//     while t < 10. * PI { // 2e5
//         evolveStep(&mut pos, &mut vel, &rad, rho, &mut acc, &mut jerk, dt);
//         if !pos[0].is_finite() {
//             pos[0].print();
//             panic!("Got non-finite value for position of particle 0 at t = {}", t);
//         }

//         for c in 0..pos.len() {
//             g.draw_point(pos[c].0, pos[c].1, 'o' as u64, (c % 4) as i16);

//         }
//         g.refresh();

//         graphics::sleep(10000);

//         t += dt;
//     }

//     pos[1].print();
//     vel[1].print();
// }

fn state_dump(pos: &Vec<Vector>, vel: &Vec<Vector>, t: f64, first: bool, rad: &Vec<f64>, rho: f64, acc: &Vec<Vector>) {
    if !first {
        eprintln!(",");
    }
    let KEs = kinetic_energy(vel, rad, rho);
    let PEs = potential_energy2(pos, rad, rho);
    eprintln!("\t{{");
    eprintln!("\t\t\"time\": {:e},", t);
    eprintln!("\t\t\"states\": [");
    for i in 0..pos.len() {
        eprintln!("\t\t\t{{");
        eprintln!("\t\t\t\t\"displacement\": [{:e}, {:e}, {:e}],", pos[i].0, pos[i].1, pos[i].2);
        eprintln!("\t\t\t\t\"velocity\": [{:e}, {:e}, {:e}],", vel[i].0, vel[i].1, vel[i].2);
        eprintln!("\t\t\t\t\"KE\": {:e},", KEs[i]);
        eprintln!("\t\t\t\t\"PE\": {:e},", PEs[i]);
        eprintln!("\t\t\t\t\"acceleration\": [{:e}, {:e}, {:e}]", acc[i].0, acc[i].1, acc[i].2);

        if i+1 < pos.len() {
            eprintln!("\t\t\t}},");
        }
        else {
            eprintln!("\t\t\t}}");
        }
        
    }
    eprintln!("\t\t]");
    eprint!("\t}}");
}

pub fn main_collisions() {
    // collision test
    let tmp_dt = 0.0001 * 2. * PI;

    let test = TestSetup::new(1e-7, tmp_dt, 1e-7, 1e-7, 0.1, false, KBCalculator::LEWIS);
    let result = match run_test(&test, true) {
        Ok(result) => result,
        Err((why, _)) => panic!(why)
    };
    result.1.print();
    println!("Test");
    
    let mut out = CSVOutput::new("data/result.csv");
    out.writeHeader();
    out.writeEntry(&test, &result.0, &result.1);
}

pub fn run_test(test: &TestSetup, print_debug: bool) -> Result<(TestData, TestResult), (String, TestData)> {
    let mut testData = TestData::new(&test);

    // println!("{} {}", sigmoid(0.), sigmoid(-test.r1 * test.sig_c));

    if print_debug {
        println!("b = {:e}, k = {:e}", test.b, test.k);
        print!("Particle 1 init:\n  Position = ");
        testData.pos[1].print();
        print!("  Velocity = ");
        testData.vel[1].print();
    }

    let mut g = if test.do_graphics {
        graphicsM!(1e-6)
    }
    else {
        graphics::Graphics::dummy()
    };

    let dataPoints = 10000;
    let spacing = test.max_time / (dataPoints - 1) as f64;

    if let Err(why) = testData.collisionUpdate() {
        return Err((why, testData));
    }

    if test.do_state_dump {
        eprintln!("INIT");
        eprintln!("{{");
        eprintln!("\t\"dt\": {:e},", test.dt);
        eprintln!("\t\"rho\": {:e},", test.rho);
        eprintln!("\t\"data\": [");
        state_dump(&testData.pos, &testData.vel, 0., true, &testData.rad, test.rho, &testData.acc);
    }

    let mut firstTime = true;

    calcAccJerk(&mut testData, &test);
    let mut t = 0.;
    let mut t_spacer = 0.;
    while t < test.max_time && !testData.isDone() { // 2e5

        if t_spacer > spacing || true {
            if test.do_state_dump {
                state_dump(&testData.pos, &testData.vel, t, false, &testData.rad, test.rho, &testData.acc);
            }
            t_spacer = 0.;
        }

        match test.integrator {
            Integrator::Jerk => {
                evolveStep(&mut testData, &test);
            }
            Integrator::KickStepKick => {
                evolveStepKickStepKick(&mut testData, &test);
            }
        }
        
        testData.requireFinite();
        // test analysis
        if let Err(why) = testData.collisionUpdate() {
            return Err((why, testData));
        }

        if let CollisionPhase::Colliding = testData.phase {
            if firstTime && print_debug {
                firstTime = false;
                println!("Debug");
                println!("{:e}", (testData.pos[1] - testData.pos[0]).mag() - test.r0 - test.r1);
                testData.vel[0].print();
                testData.acc[0].print();
            }
        }

        if test.do_graphics {
            for c in 0..testData.pos.len() {
                g.draw_point(testData.pos[c].0, testData.pos[c].1, 'o' as u64, (c % 4) as i16);
            }
            g.refresh();
            graphics::sleep(1000);
        }

        t += test.dt;
        t_spacer += test.dt;
    }
    if !testData.isDone() {
        if testData.isColliding() {
            return Err((String::from("test failed - collision still ongoing"), testData));
        }
        else {
            return Err((format!("test failed - collision never started"), testData));
        }
        
    }

    if test.do_state_dump {
        state_dump(&testData.pos, &testData.vel, t, false, &testData.rad, test.rho, &testData.acc);
        eprintln!("\n\t]");

        if test.do_graphics {
            g.end();
        }

        eprintln!("}}");
    }

    let result = TestResult::new(&testData, &test, t);

    if print_debug {
        print!("Particle 1 final:\n  Position = ");
        testData.pos[1].print();
        print!("  Velocity = ");
        testData.vel[1].print();
    }

    return Ok((testData, result));
}