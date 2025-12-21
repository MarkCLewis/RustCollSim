// An EventForce implementation for gravity plus soft-sphere collisions.

use std::collections::HashMap;

use crate::{design::{single_particle_event_force::EventForce, system::Particle}, vectors::Vector};

pub struct GravityAndSoftSphereEventForce {
  collision_velocity: Vec<HashMap<usize, f64>>,
}

impl GravityAndSoftSphereEventForce {
  pub fn new(n: usize) -> GravityAndSoftSphereEventForce {
    let collision_velocity = vec![HashMap::<usize, f64>::new(); n];
    GravityAndSoftSphereEventForce { collision_velocity }
  }
}

impl EventForce for GravityAndSoftSphereEventForce {
  type SingleParticleData = HashMap<usize, f64>;

  fn get_all_particle_data(&mut self) -> Vec<Self::SingleParticleData> {
    std::mem::replace(&mut self.collision_velocity, vec![])
  }

  fn set_all_particle_data(&mut self, spds: Vec<Self::SingleParticleData>) {
    self.collision_velocity = spds
  }

  fn get_one_particle_data(&mut self, index: usize) -> Self::SingleParticleData {
    std::mem::replace(&mut self.collision_velocity[index], HashMap::new())
  }

  fn set_one_particle_data(&mut self, index: usize, spd: Self::SingleParticleData) {
    self.collision_velocity[index] = spd
  }

  fn particle_particle(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, spd: &mut Self::SingleParticleData, dt: f64) -> (f64, Vector) {
    // Check separattion
    let dx = p1.x - p2.x;
    let dist = dx.mag();
    if dist > p1.r + p2.r {
      // Calculate gravity
      // TODO:

      // Clear impact velocity if it exists
      spd.remove(&i2);
    } else {
      // Calculate collision force
      let dv = p1.v - p2.v;
      let vel = dv.mag();
      let impact_vel = f64::max(vel, *spd.get(&i2).unwrap_or(&0.0));
      // TODO: Force calculation

      // Set impact velocity
      spd.insert(i2, impact_vel);
    }
    (0.001, Vector::ZERO)
  }

  fn particle_group(&self, i1: usize, p1: &Particle, cm_x: &Vector, cm_m: f64, dt: f64) -> (f64, Vector) {
    let dx = p1.x - *cm_x;
    let dist = dx.mag();
    let mag = p1.m * cm_m / (dist * dist * dist);
    (dt, dx * mag)
  }
}