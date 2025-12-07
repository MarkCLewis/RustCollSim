// A force that represents the Hill's forces for linearized ring simulations.

use rayon::iter::{IntoParallelRefMutIterator, ParallelIterator};

use crate::{design::system::{Force, Particle}, vectors::Vector};

pub struct HillsForce {
  dt: f64,
  n: f64,
  kappa: f64,
  n_z: f64,
}

impl HillsForce {
  pub fn new(dt: f64) -> Self {
    Self {
      dt,
      n: 1.0,
      kappa: 1.0,
      n_z: 1.0,
    }
  }

  pub fn apply_for_one(&self, p: &mut Particle, dt: f64) {
      let ax = 2. * self.n * p.v.y() - (self.kappa * self.kappa - 4. * self.n * self.n) * p.x.x();
      let ay = -2. * self.n * p.v.x();
      let az = -self.n_z * self.n_z * p.x.z();

      let dv = Vector::new(ax, ay, az) * dt;
      p.v += dv;
  }
}


impl Force for HillsForce {
  fn apply_force(&self, pop: &mut [Particle]) {
    pop.par_iter_mut().for_each(|p| {
      self.apply_for_one(p, self.dt);
    });
  }
}