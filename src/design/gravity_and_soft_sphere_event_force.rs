// An EventForce implementation for gravity plus soft-sphere collisions.

struct GravityAndSoftSphereEventForce {
  
}

impl EventForce for GravityAndSoftSphereEventForce {
  fn particle_particle(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, dt: f64) -> (f64, Vector) {

  }

  fn particle_group(&self, i1: usize, p1: &Particle, g: &Vector, m: f64, dt: f64) -> (f64, Vector) {
    
  }
}