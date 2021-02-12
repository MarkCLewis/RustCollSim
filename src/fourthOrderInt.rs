use crate::data::basic::*;
use crate::graphicsM;
use crate::graphics;
use std::f64::consts::PI;


// for r, v_0 = 1e-7, rho = 0.88
// b = 3.4194745456729856e-20, k = 6.595530918688126e-18
const B: f64 = 3.4194745456729856e-20;
const K: f64 = 6.595530918688126e-18;


fn sigmoid(x: f64) -> f64 {
    // sigmoid(100) = 1.0
    // sigmoid(-100) = 3.7200759760208356e-44 ~ 0
    if x > 100. { 1. }
    else if x < -100. { 0. }
    else {
        1.0 / (1.0 + (-x).exp())
    }
}

fn sigmoidP(x: f64) -> f64 {
    // NOTE: likely incorrect derivative!

    // there would be a negative sign, but this func is even 
    // sigmoidP(+-100) = 3.7200759760208356e-44 ~ 0
    if x.abs() > 100.0 {
        return 0.0;
    }
    let denom: f64 = x.exp() + 1.0;
    return x.exp() / (denom * denom);
}


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


            //println!("{} {} {} {} {} {}", rji.is_finite(), vji.is_finite(), r2, rv_r2, r, r3);

            let da = rji / rCube;
            let dj = (vji - rji * 3. * rv_r2) / rCube;
            let massi = 4. * PI * rho / 3. * rad[i] * rad[i] * rad[i];
            let massj = 4. * PI * rho / 3. * rad[j] * rad[j] * rad[j];

            acc[i] += da * massj;
            acc[j] -= da * massi;


            jerk[i] += dj * massj;
            jerk[j] -= dj * massi;

            let penDepth = r - (rad[i] + rad[j]);
            if penDepth < 0. {
                // collision!
                let beta = B / ((massi + massj) / 2.);
                let omega_0_sq = K / ((massi + massj) / 2.);
                // 
                let springAcc = -omega_0_sq * penDepth;

                let rji_unit = rji.unit_vector(); // points to j from i

                acc[i] += rji_unit * -springAcc; // should point away from j
                acc[j] += rji_unit * springAcc; // should point towards j

                let dragji = vji * beta; // vji is relative velocity
                // points to j
                let dragij = dragji * -1.;
                // points to i

                acc[i] += dragij; // dragij points to i, so this should slow it down
                acc[j] += dragji; // dragji points to j, so this should slow it down

                // spring acceleration always is outwards, so if the relative vel is opposite to the other body, we still want to push away from
                // the other body
                let vji_proj_on_rji = rji * vji.dot(&rji_unit).abs(); // project relative velocity on the vector pointing from i to j

                // a + beta * v + omegaSq * x = 0
                // d/dt
                // jerk + beta * acc + omegaSq * v = 0
                // jerk[i]
                // jerk[j]

                // part 1 
                // jerk += beta * acc // how to???

                // part 2
                // omegaSq * v
                jerk[i] += vji_proj_on_rji * -omega_0_sq; // signs might be the wrong way around
                jerk[j] += vji_proj_on_rji * omega_0_sq;
            }
        }
    }

    for i in 0..(pos.len()) {
        for j in (i+1)..(pos.len()) {
            let rji = pos[j] - pos[i]; // relative pos
            let rSq = rji.dot(&rji);
            let r = rSq.sqrt();

            let penDepth = r - (rad[i] + rad[j]);
            if penDepth < 0. {
                let massi = 4. * PI * rho / 3. * rad[i] * rad[i] * rad[i];
                let massj = 4. * PI * rho / 3. * rad[j] * rad[j] * rad[j];

                let beta = B / ((massi + massj) / 2.);
                let aji = acc[j] - acc[i];

                // collision!
                // part 1 
                // jerk += beta * acc
                jerk[i] += aji * -beta;
                jerk[j] += aji * beta;
            }
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
    let dt = 0.001 * 2. * PI;
    let mut pos: Vec<Vector> = vec!(Vector(0.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0));
    let mut vel: Vec<Vector> = vec!(Vector(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0));
    let rad: Vec<f64> = vec!(1e-7, 1e-14);
    let rho: f64 = 3.0 / (4.0 * PI *rad[0]*rad[0]*rad[0]); // what?
    let mut acc: Vec<Vector> = vec!(Vector(0., 0., 0.), Vector(0., 0., 0.));
    let mut jerk: Vec<Vector> = vec!(Vector(0., 0., 0.), Vector(0., 0., 0.));

    pos[1].print();
    vel[1].print();

    let mut g = graphicsM!();

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