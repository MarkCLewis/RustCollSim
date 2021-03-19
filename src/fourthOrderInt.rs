use crate::data::basic::*;
use crate::graphicsM;
use crate::graphics;
use std::f64::consts::PI;

// for r, v_0 = 1e-7, rho = 0.88
// b = 3.4194745456729856e-20, k = 6.595530918688126e-18
const B: f64 = 3.4194745456729856e-20;
const K: f64 = 6.595530918688126e-18;

const C: f64 = 1.0; // sigmoid modifier

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

// d sig (x(t))
// dt    

// fn sigmoid_dot(x: f64, )

// fn sigmoidP(x: f64) -> f64 {
//     // NOTE: likely incorrect derivative!

//     // there would be a negative sign, but this func is even 
//     // sigmoidP(+-100) = 3.7200759760208356e-44 ~ 0
//     if x.abs() > 100.0 {
//         return 0.0;
//     }
//     let denom: f64 = x.exp() + 1.0;
//     return x.exp() / (denom * denom);
// }

pub fn calcAccJerk(pos: &Vec<Vector>, vel: &Vec<Vector>, rad: &Vec<f64>, rho: f64, acc: &mut Vec<Vector>, jerk: &mut Vec<Vector>) {

    for i in 0..(pos.len()) {
        acc[i] = Vector(0., 0., 0.);
        jerk[i] = Vector(0., 0., 0.);
    }

    for i in 0..(pos.len()) {
        for j in (i+1)..(pos.len()) {
            let rji = pos[j] - pos[i]; // relative pos
            let vji = vel[j] - vel[i]; // relative vel

            let rSq = rji.dot(&rji);
            let r = rSq.sqrt();
            let rCube = r * rSq;

            let rv_r2 = rji.dot(&vji) / rSq;

            let x_hat = rji / r;
            let delta = r - (rad[i] + rad[j]);

            let delta_dot = x_hat.dot(&vji);

            let x_hat_dot = (vji - x_hat * vji.dot(&x_hat)) / r;

            //println!("{} {} {} {} {} {}", rji.is_finite(), vji.is_finite(), r2, rv_r2, r, r3);

            let da = rji / rCube;
            let dj = (vji - rji * 3. * rv_r2) / rCube;
            let massi = 4. * PI * rho / 3. * rad[i] * rad[i] * rad[i];
            let massj = 4. * PI * rho / 3. * rad[j] * rad[j] * rad[j];

            let sig_pos = sigmoid(delta * C);
            let sig_neg = sigmoid(-delta * C);

            let sig_dot_pos = sigmoidDot(delta_dot * C, delta * C);
            let sig_dot_neg = sigmoidDot(-delta_dot * C, -delta * C);

            //if delta > 0. {
                // no collision
            
            let ai_g = da * massj;
            let aj_g = da * massi;
            
            let ji_g = dj * massj;
            let jj_g = dj * massi;
            
            // gravity
            acc[i] += ai_g * sig_pos;
            acc[j] -= aj_g * sig_pos;

            jerk[i] += ji_g * sig_dot_pos + ai_g * sig_pos; // TODO: ????
            jerk[j] -= jj_g * sig_dot_pos + ai_g * sig_pos;
            // mj = sig dot * force + sig * jerk

            //}
            //else {//if delta < 0. {
                // collision!
            
            // collision
            let f_spring = x_hat * -K * delta;
            let f_damp = vji * -B;

            let f_total = f_spring + f_damp;

            // F = m_i * a_i
            // a_i = F / m_i
            let ai = f_total / massi;
            let aj = f_total / massj;
            let aji = aj - ai;

            // someone somewhere suggested this as d/dt F
            let yank_spring = (x_hat * delta_dot + x_hat_dot * delta) * -K;
            let yank_damp = aji * -B;

            let yank_total = yank_spring + yank_damp;

            // F = m_i * a_i
            // d/dt F = m_i * jerk_i
            // jerk_i = d/dt F / m_i

            let ji = yank_total / massi;
            let jj = yank_total / massj;

            acc[i] -= ai * sig_neg;
            acc[j] += aj * sig_neg;

            // jerk[i] -= ji; // TODO: ???
            // jerk[j] += jj;
            // mj = sig dot * force + sig * jerk
            jerk[i] -= ai * sig_dot_neg + ji * sig_neg;
            jerk[j] += aj * sig_dot_neg + jj * sig_neg;
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

pub fn evolveStep(pos: &mut Vec<Vector>, vel: &mut Vec<Vector>, rad: &Vec<f64>, rho: f64, 
    acc: &mut Vec<Vector>, jerk: &mut Vec<Vector>, dt: f64) {
    // TODO: This isn't ideally efficient. Better to keep two vectors around and reuse, but it will do for now.
    let oldPos = pos.clone();
    let oldVel = vel.clone();
    let oldAcc = acc.clone();
    let oldJerk = jerk.clone();

    predictStep(pos, vel, acc, jerk, dt);
    //assert_eq!(pos[0].is_finite(), true);
    calcAccJerk(pos, vel, rad, rho, acc, jerk);
    //assert_eq!(pos[0].is_finite(), true);
    correctStep(pos, vel, acc, jerk, &oldPos, &oldVel, &oldAcc, &oldJerk, dt);
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

pub fn evolveStepKickStepKick(pos: &mut Vec<Vector>, vel: &mut Vec<Vector>, rad: &Vec<f64>, rho: f64, 
    acc: &mut Vec<Vector>, jerk: &mut Vec<Vector>, dt: f64) {
    // TODO: This isn't ideally efficient. Better to keep two vectors around and reuse, but it will do for now.

    // this assumes acc has already been calculated

    integrate_kick_step_kick_1(pos, vel, acc, dt);

    calcAccJerk(pos, vel, rad, rho, acc, jerk);

    integrate_kick_step_kick_2(vel, acc, dt);
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

pub fn main() {
    // gravity test
    let dt = 0.001 * 2. * PI;
    let mut pos: Vec<Vector> = vec!(Vector(0.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0));
    let mut vel: Vec<Vector> = vec!(Vector(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0));
    let rad: Vec<f64> = vec!(1e-7, 1e-14);
    let rho: f64 = 3.0 / (4.0 * PI *rad[0]*rad[0]*rad[0]); // what?
    let mut acc: Vec<Vector> = vec!(Vector(0., 0., 0.), Vector(0., 0., 0.));
    let mut jerk: Vec<Vector> = vec!(Vector(0., 0., 0.), Vector(0., 0., 0.));

    pos[1].print();
    vel[1].print();

    let mut g = graphicsM!(1e-0);

    calcAccJerk(&pos, &vel, &rad, rho, &mut acc, &mut jerk);
    let mut t = 0.;
    while t < 10. * PI { // 2e5
        evolveStep(&mut pos, &mut vel, &rad, rho, &mut acc, &mut jerk, dt);
        if !pos[0].is_finite() {
            pos[0].print();
            panic!("Got non-finite value for position of particle 0 at t = {}", t);
        }

        for c in 0..pos.len() {
            g.draw_point(pos[c].0, pos[c].1, 'o' as u64, (c % 4) as i16);

        }
        g.refresh();

        graphics::sleep(10000);

        t += dt;
    }

    pos[1].print();
    vel[1].print();
}

fn state_dump(pos: &Vec<Vector>, vel: &Vec<Vector>, t: f64, first: bool, rad: &Vec<f64>, rho: f64) {
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
        eprintln!("\t\t\t\t\"PE\": {:e}", PEs[i]);

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
    let dt = 0.001 * 2. * PI;
    //let mut pos: Vec<Vector> = vec!(Vector(-1e-6, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
    //let mut vel: Vec<Vector> = vec!(Vector(0.1e-7, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
    let mut pos: Vec<Vector> = vec!(Vector(-1.1e-7, 0.0, 0.0), Vector(1.1e-7, 0.0, 0.0));
    let mut vel: Vec<Vector> = vec!(Vector(0.5e-6, 0.0, 0.0), Vector(-0.5e-6, 0.0, 0.0));
    let rad: Vec<f64> = vec!(1e-7, 1e-7);
    let rho: f64 = 0.88; // 3.0 / (4.0 * PI *rad[0]*rad[0]*rad[0]); // what is this line mean?
    let mut acc: Vec<Vector> = vec!(Vector(0., 0., 0.), Vector(0., 0., 0.));
    let mut jerk: Vec<Vector> = vec!(Vector(0., 0., 0.), Vector(0., 0., 0.));

    pos[1].print();
    vel[1].print();

    eprintln!("INIT");
    eprintln!("{{");
    eprintln!("\t\"dt\": {:e},", dt);
    eprintln!("\t\"rho\": {:e},", rho);

    let mut g = graphicsM!(1e-6);

    let dataPoints = 1000;
    let timeTo = 0.5 * PI;

    let spacing = timeTo / (dataPoints - 1) as f64;

    eprintln!("\t\"data\": [");
    state_dump(&pos, &vel, 0., true, &rad, rho);

    calcAccJerk(&pos, &vel, &rad, rho, &mut acc, &mut jerk);
    let mut t = 0.;
    let mut t_spacer = 0.;
    while t < timeTo { // 2e5
        if t_spacer > spacing {
            state_dump(&pos, &vel, t, false, &rad, rho);
            t_spacer = 0.;
        }
        

        evolveStepKickStepKick(&mut pos, &mut vel, &rad, rho, &mut acc, &mut jerk, dt);
        if !pos[0].is_finite() {
            pos[0].print();
            panic!("Got non-finite value for position of particle 0 at t = {}", t);
        }

        for c in 0..pos.len() {
            g.draw_point(pos[c].0, pos[c].1, 'o' as u64, (c % 4) as i16);

        }
        g.refresh();

        graphics::sleep(1000);

        t += dt;
        t_spacer += dt;
    }
    state_dump(&pos, &vel, t, false, &rad, rho);
    eprintln!("\n\t]");

    g.end();

    eprintln!("}}");

    pos[1].print();
    vel[1].print();
}