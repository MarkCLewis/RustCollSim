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
  fn setup(&mut self, parts: &[Particle]) {}

  fn for_one<F: EventForce>(&self, i1: usize, p1: &Particle, spd1: &mut F::SingleParticleData, force: &F, dt: f64, parts: &[Particle]) -> (f64, Vector) {
    let mut min_time = 1e100;
    let mut dv = Vector ([0.0, 0.0, 0.0]);
    parts.iter().enumerate().for_each(|t| {
      let (i2, p2) = t;
      if i2 != i1 {
        let (t, delta_v) = force.particle_particle(i1, p1, i2, p2, spd1, dt);
        min_time = f64::min(min_time, t);
        dv += delta_v;
      }
    });
    (min_time, dv)
  }
}