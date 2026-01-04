// The goal of this component is to traverse through particles doing various work.

use crate::{forces::{single_particle_event_force::{EventForce, Traverser}}, design::system::{BoundaryCondition, Particle, Population}, vectors::Vector};

pub struct BruteForceParticleTraversal {
}

impl BruteForceParticleTraversal {
  pub fn new() -> BruteForceParticleTraversal {
    BruteForceParticleTraversal { }
  }
}

impl Traverser for BruteForceParticleTraversal {
  fn setup(&mut self, pop: &impl Population) {}

  fn accel_for_one<F: EventForce>(&self, i1: usize, p1: &Particle, spd1: &mut F::SingleParticleData, force: &F, pop: &impl Population) -> Vector {
    let mut acc_sum = Vector ([0.0, 0.0, 0.0]);
    if let Some(mirror_offsets) = pop.boundary_conditions().simple_mirror_offsets() {
      for (mirror_num, (offset_x, offset_v)) in mirror_offsets.iter().enumerate() {
        pop.particles().iter().enumerate().for_each(|t| {
          let (i2, p2_ref) = t;
          let mut p2 = p2_ref.clone();
          p2.x += *offset_x;
          p2.v += *offset_v;
          if i2 != i1 {
            let acc = force.particle_particle_accel(i1, p1, i2, &p2, spd1, mirror_num);
            acc_sum += acc;
          }
        });
      }
    } else {
      pop.particles().iter().enumerate().for_each(|t| {
        let (i2, p2_ref) = t;
        for (mirror_num, p2) in pop.boundary_conditions().mirrors(p2_ref).enumerate() {
          if i2 != i1 {
            let acc = force.particle_particle_accel(i1, p1, i2, &p2, spd1, mirror_num);
            acc_sum += acc;
          }
        }
      });
    }
    println!("for-one accel i1:{}, {}", i1, acc_sum);
    acc_sum
  }

  fn time_step_for_one<F: EventForce>(&self, i1: usize, p1: &Particle, spd1: &F::SingleParticleData, force: &F, pop: &impl Population, accs: &Vec<Vector>) -> f64 {
    let mut min_time_delta = 1e100;
    if let Some(mirror_offsets) = pop.boundary_conditions().simple_mirror_offsets() {
      for (mirror_num, (offset_x, offset_v)) in mirror_offsets.iter().enumerate() {
        pop.particles().iter().enumerate().for_each(|t| {
          let (i2, p2_ref) = t;
          let mut p2 = p2_ref.clone();
          p2.x += *offset_x;
          p2.v += *offset_v;
          if i2 != i1 {
            let t = force.particle_particle_time_step(i1, p1, i2, &p2, spd1, accs, mirror_num);
            min_time_delta = f64::min(min_time_delta, t);
          }
        });
      }
    } else {
      pop.particles().iter().enumerate().for_each(|t| {
        let (i2, p2_ref) = t;
        for (mirror_num, p2) in pop.boundary_conditions().mirrors(p2_ref).enumerate() {
          if i2 != i1 {
            let t = force.particle_particle_time_step(i1, p1, i2, &p2, spd1, accs, mirror_num);
            min_time_delta = f64::min(min_time_delta, t);
          }
        }
      });
    }
    println!("for-one time i1:{}, {}", i1, min_time_delta);
    min_time_delta
  }
}