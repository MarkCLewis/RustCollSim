// An EventForce implementation for gravity plus soft-sphere collisions.

use std::collections::HashMap;
use std::fmt::Debug;

use rayon::iter::{IndexedParallelIterator, IntoParallelRefIterator};

use crate::{debugln, design::system::Particle, forces::{ no_explode::SpringDerivation, single_particle_event_force::{EventForce, Reindexable, SingleParticleEvent} }, vectors::Vector};

pub struct GravityAndSoftSphereEventForce<SD: SpringDerivation + Sync + Send, SPD: Sync + Send + Debug + Default> {
  spring_derivation: SD,
  desired_collision_step_count: i32,
  collision_velocity: Vec<SPD>,
}

pub trait SingleParticleDataType: Sync + Send + Debug + Default + Clone {
  fn new() -> Self;
  fn check_data_for_events(next_time: f64, collision_velocity: &Vec<Self>) -> Vec<SingleParticleEvent>;
  fn combine_particle_pair_data(ppd1: Option<f64>, ppd2: Option<f64>) -> Option<f64>;
  fn update_particle_pair_data(&mut self, i1:usize, i2: usize, ppd: Option<f64>);
  fn impact_vel(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, alt_vel: &f64) -> f64;
}

impl SingleParticleDataType for HashMap<usize, f64> {
  fn new() -> Self {
    HashMap::<usize, f64>::new()
  }
  fn check_data_for_events(next_time: f64, collision_velocity: &Vec<Self>) -> Vec<SingleParticleEvent> {
    let mut events = vec![];
    collision_velocity.iter().enumerate().for_each(|t| {
      let (i1, v) = t;
      for (i2, _) in v {
        let matched = collision_velocity[*i2].contains_key(&i1);
        if !matched {
          println!("No match! {} {}", i1, i2);
          events.push(SingleParticleEvent { event_time: next_time, added_time: next_time, index: *i2 });
        }
      }
    });
    events
  }
  fn update_particle_pair_data(&mut self, i1:usize, i2: usize, ppd: Option<f64>) {
    match ppd {
      Some(vel) => {
        match self.get(&i2) {
          Some(_) => {}
          None => {
            self.insert(i2, vel);
          }
        }
      }
      None => {}
    };
  }
  fn combine_particle_pair_data(ppd1: Option<f64>, ppd2: Option<f64>) -> Option<f64> {
    match ppd1 {
      Some(v1) =>
        match ppd2 {
          Some(v2) => Some(f64::max(v1, v2)),
          None => ppd1
        },
      None => ppd2
    }
  }
  fn impact_vel(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, alt_vel: &f64) -> f64 {
    *self.get(&i2).unwrap_or(&alt_vel)
  }
}

#[derive(Debug, Default, Clone)]
pub struct RadiusImpactVelocity {}

impl SingleParticleDataType for RadiusImpactVelocity {
  fn new() -> Self {
    RadiusImpactVelocity {}
  }
  fn check_data_for_events(next_time: f64, collision_velocity: &Vec<Self>) -> Vec<SingleParticleEvent> {
    Vec::new()
  }
  fn combine_particle_pair_data(ppd1: Option<f64>, ppd2: Option<f64>) -> Option<f64> {
    None
  }
  fn update_particle_pair_data(&mut self, i1:usize, i2: usize, ppd: Option<f64>) {
  }
  fn impact_vel(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, alt_vel: &f64) -> f64 {
    p1.r + p2.r
  }
}

impl Reindexable for HashMap<usize, f64> {
  fn reindex(&self, permutation_map: &Vec<usize>) -> Self {
    let mut ret = HashMap::<usize, f64>::new();
    for (k, v) in self {
      ret.insert(permutation_map[*k], *v);
    }
    ret
  }
}

impl Reindexable for RadiusImpactVelocity {
  fn reindex(&self, permutation_map: &Vec<usize>) -> Self {
    RadiusImpactVelocity {}
  }
}

impl<SD: SpringDerivation + Sync + Send, SPDT: SingleParticleDataType> GravityAndSoftSphereEventForce<SD, SPDT> {
  pub fn new(n: usize, spring_derivation: SD, desired_collision_step_count: i32) -> GravityAndSoftSphereEventForce<SD, SPDT> {
    let collision_velocity = vec![SPDT::new(); n];
    GravityAndSoftSphereEventForce { spring_derivation, desired_collision_step_count, collision_velocity }
  }

  fn collision_time_step(&self, k: f64, m: f64, b: f64) -> f64 {
    use crate::forces::no_explode::omega_l;
    let omega_l = omega_l(k, m, b);

    // T = 1/f = 2\pi/\omega
    // Time of collision is T/2
    let collision_time = std::f64::consts::PI / omega_l;
    collision_time / self.desired_collision_step_count as f64
  }

  fn safe_collision_time_step(&self, vel: f64, acc: f64, r1: f64, r2: f64) -> f64 {
    let max_ok_pen_estimate = f64::max(r1, r2) * self.spring_derivation.get_pen_fraction() / self.desired_collision_step_count as f64;
    let root = vel * vel - 4.0 * acc * -(max_ok_pen_estimate * 2.0);
    // The positive root is >0.0, so that is the one we want.
    (-vel + f64::sqrt(root)) / (2.0 * acc)
  }

  fn gravity_time_step(&self, separation_distance: f64, vel: f64, acc: f64) -> f64 {
    // Velocity, acceleration, and distance are always positive (magnitudes). So the root is always positive.
    let root = vel * vel - 4.0 * acc * -separation_distance;
    // The positive root is >0.0, so that is the one we want.
    let collision_time = (-vel + f64::sqrt(root)) / (2.0 * acc);
    collision_time / 2.0
  }

  fn safe_motion_time_step(&self, separation_distance: f64, relative_speed_estimate: f64, acc: f64, r1: f64, r2: f64) -> f64 {
    // how far two particles can intersect without things getting out of hand
    let max_ok_pen_estimate = f64::max(r1, r2) * self.spring_derivation.get_pen_fraction() / self.desired_collision_step_count as f64;
    let distance_for_global_speed_estimate = f64::max(separation_distance, max_ok_pen_estimate);
    // Velocity, acceleration, and distance are always positive (magnitudes). So the root is always positive.
    let root = relative_speed_estimate * relative_speed_estimate - 4.0 * acc * -distance_for_global_speed_estimate;
    // The positive root is >0.0, so that is the one we want.
    let collision_time = (-relative_speed_estimate + f64::sqrt(root)) / (2.0 * acc);
    collision_time / 2.0
  }

}

impl<SD: SpringDerivation + Sync + Send, SPDT: SingleParticleDataType + Reindexable> EventForce for GravityAndSoftSphereEventForce<SD, SPDT> {
  type SingleParticleData = SPDT;
  type ParticlePairData = f64;

  fn get_all_particle_data(&mut self) -> Vec<Self::SingleParticleData> {
    //println!("Get SPD: {:?}", self.collision_velocity);
    std::mem::replace(&mut self.collision_velocity, vec![])
  }

  fn set_all_particle_data(&mut self, spds: Vec<Self::SingleParticleData>) {
    self.collision_velocity = spds;
    //println!("Set SPD: {:?}", self.collision_velocity);
  }

  // TODO: pass in batch so we only check the particles that could have changed.
  fn check_data_for_events(&self, next_time: f64) -> Vec<SingleParticleEvent> {
    SPDT::check_data_for_events(next_time, &self.collision_velocity)
  }

  fn update_particle_pair_data(&self, i1:usize, i2: usize, spd: &mut Self::SingleParticleData, ppd: Option<Self::ParticlePairData>) {
    spd.update_particle_pair_data(i1, i2, ppd);
  }

  fn combine_particle_pair_data(&self, ppd1: Option<Self::ParticlePairData>, ppd2: Option<Self::ParticlePairData>) -> Option<Self::ParticlePairData> {
    SPDT::combine_particle_pair_data(ppd1, ppd2)
  }

  fn particle_particle_accel(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, spd: &mut Self::SingleParticleData, mirror_num: usize) -> (Vector, Option<Self::ParticlePairData>) {
    let dx = p2.x - p1.x;
    let dist = dx.mag();
    let separation_distance = dist - (p1.r + p2.r);
    let close_print = false && separation_distance < 0.2 * f64::min(p1.r, p2.r);
    // if separation_distance < -10.0* p1.r * self.spring_derivation.get_pen_fraction() {
    //   println!("Bad Data i1={}, i2={}, mirror={}, x1={}, x2={}, dx={}, dist={:e}, sep={:e}", i1, i2, mirror_num, p1.x, p2.x, dx, dist, separation_distance);
    // }
    if close_print {
      println!("p:p i1:{} i2:{} mirror:{} dx:{} dist:{:e} sep_dist:{:e}", i1, i2, mirror_num, dx, dist, separation_distance);
    }
    if separation_distance > 0.0 {
      // Calculate gravity
      let mag = p2.m / (dist * dist * dist);
      // Calculate event time delta
      if close_print { println!("gravity {}", dx*mag); }
      (dx * mag, None)
    } else {
      let dv = p2.v - p1.v;
      let vel = dv.mag();
      let last_impact_vel = spd.impact_vel(i1, p1, i2, p2, &0.0);
      let impact_vel = if last_impact_vel == 0.0 { vel } else { last_impact_vel };
      // TODO: consider a minimum velocity if things get too squishy.
      let reduced_mass = (p1.m * p2.m) / (p1.m + p2.m);
      let (b, k) = self.spring_derivation.b_and_k(impact_vel, reduced_mass, f64::max(p1.r, p2.r));
      if close_print { println!("Colliding, i1:{}, i2:{}, dist = {:e}, k={:e}, b={:e}", i1, i2, dist, k, b); }
      // Calculate collision force
      let f_spring = (dx / dist) * k * separation_distance;
      let f_damp = dv * b * dv.dot(&dx).abs()/vel/dist;
      if close_print { println!("spring {:e} {:e} {} {} {}", k, b, f_spring, f_damp, (f_spring + f_damp) / p1.m); }

      ((f_spring + f_damp) / p1.m, Some(impact_vel))
    }
  }

  fn particle_particle_time_step(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, spd: &Self::SingleParticleData, accels: &Vec<Vector>, mirror_num: usize) -> f64 {
        // Check separattion
    let dx = p2.x - p1.x;
    let dist = dx.mag();
    let dv = p2.v - p1.v;
    let separation_distance = dist - (p1.r + p2.r);
    let close_print = false && separation_distance < 0.2 * f64::min(p1.r, p2.r);
    let vel = dv.mag();
    let impact_vel = spd.impact_vel(i1, p1, i2, p2, &vel);
    let reduced_mass = (p1.m * p2.m) / (p1.m + p2.m);
    let (b, k) = self.spring_derivation.b_and_k(impact_vel, reduced_mass, f64::max(p1.r, p2.r));
    let collision_time_step = self.collision_time_step(k, reduced_mass, b);
    let acc = (accels[i1] - accels[i2]).mag();
    if close_print {
      println!("p:p i1:{} i2:{} mirror:{} dx:{} dist:{:e} dv:{} sep_dist:{:e} vel:{:e} impact_vel: {:e} collision_step_time:{} ratio:{}", i1, i2, mirror_num, dx, dist, dv, separation_distance, vel, impact_vel, collision_time_step, separation_distance / (f64::max(p1.r, p2.r) * self.spring_derivation.get_pen_fraction()));
    }
    if separation_distance > 0.0 {
      // Calculate gravity
      let gravity_time_step = self.gravity_time_step(separation_distance, vel, acc);
      let relative_speed_estimate = 3.0 * (p1.r + p2.r);
      let safe_motion_time_step = self.safe_motion_time_step(separation_distance, relative_speed_estimate, acc, p1.r, p2.r);
      if close_print {
        println!("acc:{:e} gravity_time_step:{} safe_motion_time_step:{}", acc, gravity_time_step, safe_motion_time_step);
      }
      f64::min(safe_motion_time_step, f64::max(collision_time_step, gravity_time_step))
    } else {
      let safe_collision_time_step = self.safe_collision_time_step(vel, acc, p1.r, p2.r);
      if close_print { println!("i1:{} i2:{} safe_collision_time_step:{}", i1, i2, safe_collision_time_step); }
      f64::min(collision_time_step, safe_collision_time_step)
    }
  }

  fn particle_group_accel(&self, _i1: usize, p1: &Particle, cm_x: &Vector, cm_m: f64) -> Vector {
    let dx = *cm_x - p1.x;
    let dist = dx.mag();
    let mag = cm_m / (dist * dist * dist);
    dx * mag
  }
}

#[cfg(test)]
mod test {
  use std::collections::HashMap;

  use crate::{forces::gravity_and_soft_sphere_event_force::GravityAndSoftSphereEventForce, forces::no_explode::Rotter};

  #[test]
  fn check_next_time() {
    let force = GravityAndSoftSphereEventForce::<Rotter, HashMap<usize, f64>>::new(2, Rotter::new(0.5, 0.01), 20);
    
    let k = 1.0;
    let b = 1.0;
    let m = 1.0;

    let expected_coll_time = 3.6275987284684357 / 20.0;
    let expected_gravity_time = 0.5 * (-1.0 + f64::sqrt(1.0 + 4.0)) / 2.0;
    let expected_safe_time = 0.25; // TODO: put root formula here

    let check_time_separate = force.collision_time_step(k, m, b);
    assert_eq!(check_time_separate, expected_coll_time);

    let check_time_overlap = force.gravity_time_step(1.0, 1.0, 1.0);
    assert_eq!(check_time_overlap, expected_gravity_time);

    let check_time_overlap = force.safe_motion_time_step(1.0, 1.5, 1.0, 1.0, 1.0);
    assert_eq!(check_time_overlap, expected_safe_time);
  }

  #[test]
  fn check_particle_particle() {

  }
}
