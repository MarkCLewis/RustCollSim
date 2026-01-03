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

  fn for_one<F: EventForce>(&self, i1: usize, p1: &Particle, spd1: &mut F::SingleParticleData, force: &F, dt: f64, pop: &impl Population) -> (f64, Vector) {
    let mut min_time_delta = 1e100;
    let mut acc_sum = Vector ([0.0, 0.0, 0.0]);
    if let Some(mirror_offsets) = pop.boundary_conditions().simple_mirror_offsets() {
      for (mirror_num, (offset_x, offset_v)) in mirror_offsets.iter().enumerate() {
        pop.particles().iter().enumerate().for_each(|t| {
          let (i2, p2_ref) = t;
          let mut p2 = p2_ref.clone();
          p2.x += *offset_x;
          p2.v += *offset_v;
          if i2 != i1 {
            let (t, acc) = force.particle_particle(i1, p1, i2, &p2, spd1, mirror_num, dt);
            min_time_delta = f64::min(min_time_delta, t);
            acc_sum += acc;
          }
        });
      }
    } else {
      pop.particles().iter().enumerate().for_each(|t| {
        let (i2, p2_ref) = t;
        for (mirror_num, p2) in pop.boundary_conditions().mirrors(p2_ref).enumerate() {
          if i2 != i1 {
            let (t, acc) = force.particle_particle(i1, p1, i2, &p2, spd1, mirror_num, dt);
            min_time_delta = f64::min(min_time_delta, t);
            acc_sum += acc;
          }
        }
      });
    }
    println!("Check time: {} {} {}", min_time_delta, p1.time, dt);
    if min_time_delta + p1.time > dt {
      min_time_delta = dt - p1.time;
    }
    if min_time_delta < 1e-12 * dt {
      println!("Warning! Changing step size to {:e} from {:e} for {} at {}", min_time_delta, 1e-12 * dt, i1, p1.time);
      min_time_delta = 1e-12 * dt;
    }
    println!("for-one i1:{}, {} {}", i1, min_time_delta, acc_sum);
    (min_time_delta, acc_sum * min_time_delta)
  }
}