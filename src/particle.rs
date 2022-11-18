use core::fmt;
use std::{f64::consts::PI, hash::Hash};

use crate::{debugln, vectors::Vector};

/// A wrapper for increased type safety
#[derive(Clone, Debug, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ParticleIndex(pub usize);

impl fmt::Display for ParticleIndex {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "index({})", self.0)
    }
}

#[derive(Clone, Debug, Copy)]
pub struct Particle {
    pub p: [f64; 3],
    pub v: [f64; 3],
    pub r: f64,
    pub m: f64,
    pub t: f64, // current time of particle
}

impl Particle {
    // fn m(&self) -> f64 {
    //     return RHO * self.r * self.r * self.r;
    // }

    pub fn mass_from_radius(r: f64, rho: f64) -> f64 {
        4. / 3. * PI * r * r * r * rho
    }

    /// returns speed (absolute)
    pub fn impact_speed(&self, other: &Self) -> f64 {
        let unit_to_p2 = (Vector(other.p) - Vector(self.p)).unit_vector(); // unit vec from p1 pointing at p2
        debugln!(
            "{unit_to_p2} {} {} {}",
            Vector(other.v),
            Vector(self.v),
            ((Vector(other.v) - Vector(self.v)) * unit_to_p2).abs()
        );
        // FIXME: velocity is crazy
        // (vel of p2 rel to p1) dot (unit vector pointing at p2 from p1)
        ((Vector(other.v) - Vector(self.v)) * unit_to_p2).abs()
    }

    pub fn apply_dv(&mut self, dv: Vector) {
        self.v = (Vector(self.v) + dv).0;
    }
}

pub fn momentum(pop: &Vec<Particle>) -> Vector {
    pop.iter()
        .map(|p| Vector(p.v) * p.m)
        .reduce(|a, b| a + b)
        .unwrap_or(Vector::ZERO)
}

pub fn kinetic_energy(pop: &Vec<Particle>) -> f64 {
    pop.iter()
        .map(|p| Vector(p.v) * Vector(p.v) * p.m / 2.)
        .sum()
}

pub fn potential_energy(pop: &Vec<Particle>) -> f64 {
    // potential is -G m1 m2 / r -> G=1
    // sum all pairs - this does n1 * n2 and n2 * n1, so divide by 2
    pop.iter()
        .map(|p1| {
            pop.iter()
                .map(|p2| {
                    if p1.p == p2.p {
                        0. // same particle
                    } else {
                        -(p1.m * p2.m) / (Vector(p1.p) - Vector(p2.p)).mag()
                    }
                })
                .sum::<f64>()
        })
        .sum::<f64>()
        / 2.
}

pub fn energy(pop: &Vec<Particle>) -> f64 {
    potential_energy(pop) + kinetic_energy(pop)
}

#[allow(dead_code)]
pub fn two_bodies() -> Vec<Particle> {
    let mut bodies = Vec::new();
    bodies.push(Particle {
        p: [0.0, 0.0, 0.0],
        v: [0.0, 0.0, 0.0],
        r: 1e-3,
        m: 1.0,
        t: 0.,
    });
    bodies.push(Particle {
        p: [1.0, 0.0, 0.0],
        v: [0.0, 1.0, 0.0],
        r: 1e-4,
        m: 1e-20,
        t: 0.,
    });
    bodies
}

#[allow(dead_code)]
pub fn two_equal_bodies(r: f64, rho: f64, init_impact_v: f64, sep_dis: f64) -> Vec<Particle> {
    two_unequal_bodies(r, r, rho, init_impact_v, sep_dis)
}

#[allow(dead_code)]
pub fn two_unequal_bodies(
    r1: f64,
    r2: f64,
    rho: f64,
    mut init_impact_v: f64,
    mut sep_dis: f64,
) -> Vec<Particle> {
    let mut bodies = Vec::new();

    init_impact_v = init_impact_v.abs();
    sep_dis = sep_dis.abs();

    // Symmetric
    //       -x->
    // -      |      +
    // P1 ->  |  <- P2
    bodies.push(Particle {
        p: [-sep_dis / 2., 0.0, 0.0],
        v: [init_impact_v / 2., 0.0, 0.0],
        r: r1,
        m: Particle::mass_from_radius(r1, rho),
        t: 0.,
    });
    bodies.push(Particle {
        p: [sep_dis / 2., 0.0, 0.0],
        v: [-init_impact_v / 2., 0.0, 0.0],
        r: r2,
        m: Particle::mass_from_radius(r2, rho),
        t: 0.,
    });
    bodies
}

// pub fn simple_sim(mut bodies: Vec<Particle>, dt: f64) {
//     let mut acc = Vec::new();
//     for _ in 0..bodies.len() {
//         acc.push([0.0, 0.0, 0.0])
//     }
//     for step in 1..1000001 {
//         for i in 0..bodies.len() - 1 {
//             for j in i + 1..bodies.len() {
//                 calc_accel(i, j, &bodies[i], &bodies[j], &mut acc);
//             }
//         }
//         for i in 0..bodies.len() {
//             bodies[i].v[0] += dt * acc[i][0];
//             bodies[i].v[1] += dt * acc[i][1];
//             bodies[i].v[2] += dt * acc[i][2];
//             bodies[i].p[0] += dt * bodies[i].v[0];
//             bodies[i].p[1] += dt * bodies[i].v[1];
//             bodies[i].p[2] += dt * bodies[i].v[2];
//             acc[i] = [0.0, 0.0, 0.0];
//         }
//         if step % 10000 == 0 {
//             println!(
//                 "{} {} {} {} {}",
//                 step, bodies[1].p[0], bodies[1].p[1], bodies[1].v[0], bodies[1].v[1]
//             );
//         }
//     }
// }

// fn calc_accel(i: usize, j: usize, pi: &Particle, pj: &Particle, acc: &mut Vec<[f64; 3]>) {
//     let dx = pi.p[0] - pj.p[0];
//     let dy = pi.p[1] - pj.p[1];
//     let dz = pi.p[2] - pj.p[2];
//     let dist = f64::sqrt(dx * dx + dy * dy + dz * dz);
//     let magi = -pj.m / (dist * dist * dist);
//     acc[i][0] += dx * magi;
//     acc[i][1] += dy * magi;
//     acc[i][2] += dz * magi;
//     let magj = pi.m / (dist * dist * dist);
//     acc[j][0] += dx * magj;
//     acc[j][1] += dy * magj;
//     acc[j][2] += dz * magj;
// }

pub fn circular_orbits(n: usize) -> Vec<Particle> {
    let mut particle_buf = vec![];
    particle_buf.push(Particle {
        p: [0.0, 0.0, 0.0],
        v: [0.0, 0.0, 0.0],
        r: 0.00465047,
        m: 1.0,
        t: 0.,
    });

    for i in 0..n {
        let d = 0.1 + ((i as f64) * 5.0 / (n as f64));
        let v = f64::sqrt(1.0 / d);
        let theta = fastrand::f64() * 6.28;
        let x = d * f64::cos(theta);
        let y = d * f64::sin(theta);
        let vx = -v * f64::sin(theta);
        let vy = v * f64::cos(theta);
        particle_buf.push(Particle {
            p: [x, y, 0.0],
            v: [vx, vy, 0.0],
            m: 1e-14,
            r: 1e-7,
            t: 0.,
        });
    }
    particle_buf
}

/// acceleration for the first particle
pub fn calc_pp_accel(pi: &Particle, pj: &Particle) -> [f64; 3] {
    let dx = pi.p[0] - pj.p[0];
    let dy = pi.p[1] - pj.p[1];
    let dz = pi.p[2] - pj.p[2];
    let dp2 = dx * dx + dy * dy + dz * dz;
    let dist = f64::sqrt(dp2);
    let magi = -pj.m / (dist * dist * dist);
    //   println!("magi={}", magi[0]);
    [magi * dx, magi * dy, magi * dz]
}
