
use core_simd::*;

pub struct Particle {
  pub p: f64x4,
  pub v: f64x4,
  pub r: f64,
  pub m: f64
}

pub fn simple_sim() {
  let mut bodies = Vec::new();
  bodies.push(Particle { p: f64x4::splat(0.0), 
                         v: f64x4::splat(0.0), r: 1.0, m: 1.0 });
  bodies.push(Particle { p: f64x4::from_array([1.0, 0.0, 0.0, 0.0]), 
                         v: f64x4::from_array([0.0, 1.0, 0.0, 0.0]), r: 1e-4, m: 1e-20 });
  let dt = 1e-3 * 2.0 * std::f64::consts::PI;
  let mut acc = Vec::new();
  for _ in 0..bodies.len() { 
      acc.push([0.0, 0.0, 0.0])
  };
  for step in 1..1000001 {
      for i in 0..bodies.len()-1 {
          for j in i+1..bodies.len() {
              calc_accel(i, j, &bodies[i], &bodies[j], &mut acc);
          }
      }
      for i in 0..bodies.len() { 
          bodies[i].v[0] += dt * acc[i][0];
          bodies[i].v[1] += dt * acc[i][1];
          bodies[i].v[2] += dt * acc[i][2];
          bodies[i].p[0] += dt * bodies[i].v[0];
          bodies[i].p[1] += dt * bodies[i].v[1];
          bodies[i].p[2] += dt * bodies[i].v[2];
          acc[i] = [0.0, 0.0, 0.0];
      }
      if step % 10000 == 0 {
          println!("{} {} {} {} {}", step, bodies[1].p[0], bodies[1].p[1], bodies[1].v[0], bodies[1].v[1]);
      }
  }
}

fn calc_accel(i: usize, j: usize, pi: &Particle, pj: &Particle, acc: &mut Vec<[f64; 3]>) {
  let dx = pi.p[0] - pj.p[0];
  let dy = pi.p[1] - pj.p[1];
  let dz = pi.p[2] - pj.p[2];
  let dist = f64::sqrt(dx*dx + dy*dy + dz*dz);
  let magi = -pj.m / (dist*dist*dist);
  acc[i][0] += dx * magi;
  acc[i][1] += dy * magi;
  acc[i][2] += dz * magi;
  let magj = pi.m / (dist*dist*dist);
  acc[j][0] += dx * magj;
  acc[j][1] += dy * magj;
  acc[j][2] += dz * magj;
}
