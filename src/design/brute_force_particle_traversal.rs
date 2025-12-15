// The goal of this component is to traverse through particles doing various work.

use crate::{design::{event_force::{EventForce, Traverser}, system::{Particle, Population}}, vectors::Vector};

pub struct BruteForceParticleTraversal<'a, P: Population + Sync> {
  pop: &'a P,
}

impl<'a, P: Population + Sync> Traverser for BruteForceParticleTraversal<'a, P> {
  fn setup(&mut self) {}

  fn for_one<F: EventForce>(&self, i1: usize, p1: &Particle, force: &F, dt: f64) -> (f64, Vector) {
    let parts = self.pop.particles();
    let mut min_time = 1e100;
    let mut dv = Vector ([0.0, 0.0, 0.0]);
    parts.iter().enumerate().for_each(|t| {
      let (i2, p2) = t;
      if i2 != i1 {
        let (t, delta_v) = force.particle_particle(i1, p1, i2, p2, dt);
        min_time = f64::min(min_time, t);
        dv += delta_v;
      }
    });
    (min_time, dv)
  }
}