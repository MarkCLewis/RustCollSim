use crate::{design::{system::{BoundaryCondition, Population, Particle}}};


pub struct BasicPopulation<BC: BoundaryCondition> {
  particles: Vec<Particle>,
  boundary: BC,
}

impl<BC: BoundaryCondition> BasicPopulation<BC> {
  fn new(particles: Vec<Particle>, boundary: BC) -> Self {
    Self {
      particles,
      boundary,
    }
  }
}

impl<BC: BoundaryCondition> Population<BC> for BasicPopulation<BC> {
  fn particles(&self) -> &[Particle] {
    &self.particles[..]
  }

  fn particles_mut(&mut self) -> &mut [Particle] {
    &mut self.particles[..]
  }

  fn end_step(&mut self, dt: f64) {
    self.particles.iter_mut().for_each(|p| {
      if p.time < dt {
        p.x += p.v * (dt - p.time);
      }
      p.time = 0.0;
    });
  }

  fn apply_boundary_condition(&mut self) {
    self.particles.iter_mut().for_each(|p| {
      self.boundary.apply(p);
    });
  }
}