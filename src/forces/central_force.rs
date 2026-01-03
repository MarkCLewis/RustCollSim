// An attractive force at the origin.

use crate::design::system::{Force, Population};

pub struct CentralForce {}

impl Force for CentralForce {
  fn apply_force(&mut self, pop: &mut impl Population) {
    // TODO:
  }
}