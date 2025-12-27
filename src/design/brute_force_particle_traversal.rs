// The goal of this component is to traverse through particles doing various work.

use crate::{design::{single_particle_event_force::{EventForce, Traverser}, system::{Particle, Population}}, vectors::Vector};

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
    pop.particles().iter().enumerate().for_each(|t| {
      let (i2, p2) = t;
      if i2 != i1 {
        let (t, acc) = force.particle_particle(i1, p1, i2, p2, spd1, dt);
        min_time_delta = f64::min(min_time_delta, t);
        acc_sum += acc;
      }
    });
    println!("Check time: {} {} {}", min_time_delta, p1.time, dt);
    if min_time_delta + p1.time > dt {
      min_time_delta = dt - p1.time;
    }
    println!("for-one i1 = {}, {} {}", i1, min_time_delta, acc_sum);
    (min_time_delta, acc_sum * min_time_delta)
  }
}