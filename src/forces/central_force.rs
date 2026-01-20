// An attractive force at the origin.

use crate::design::system::{Force, Population};

pub struct CentralForce {
  dt: f64,
}

impl CentralForce {
  pub fn new(dt: f64) -> CentralForce {
    CentralForce { dt }
  }
}

impl Force for CentralForce {
  fn apply_force(&mut self, pop: &mut impl Population) {
    // for p in pop.particles_mut() {
    //   let dist = p.x.mag();
    //   let force = 0.001 / dist;
    //   p.kick(&(-p.x / dist * force));
    // }
  }
}