pub trait Population {
  fn end_step();
}

pub trait Force {
  fn apply_force<P: Population>(&mut pop: P);
}

pub struct System<P: Population, F: Force> {
  pop: P,
  force: F
}

impl System {
  fn advance(&self) {
    self.force.apply_force(self.pop);
    self.pop.end_step();
  }
}