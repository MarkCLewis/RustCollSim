// An EventForce implementation for gravity plus soft-sphere collisions.

use std::collections::HashMap;

use rayon::iter::{IndexedParallelIterator, IntoParallelRefIterator};

use crate::{debugln, design::system::Particle, forces::{ no_explode::SpringDerivation, single_particle_event_force::{EventForce, SingleParticleEvent} }, vectors::Vector};

pub struct GravityAndSoftSphereEventForce<SD: SpringDerivation + Sync + Send> {
  spring_derivation: SD,
  desired_collision_step_count: i32,
  collision_velocity: Vec<HashMap<(usize, usize), f64>>,
}

impl<SD: SpringDerivation + Sync + Send> GravityAndSoftSphereEventForce<SD> {
  pub fn new(n: usize, spring_derivation: SD, desired_collision_step_count: i32) -> GravityAndSoftSphereEventForce<SD> {
    let collision_velocity = vec![HashMap::<(usize, usize), f64>::new(); n];
    GravityAndSoftSphereEventForce { spring_derivation, desired_collision_step_count, collision_velocity }
  }

  fn get_next_time(
      &self,
      i1: usize,
      i2: usize,
      separation_distance: f64,
      current_impact_vel: f64,
      k: f64,
      m: f64,
      b: f64,
      relative_speed_estimate: f64,
      r1: f64,
      r2: f64,
  ) -> f64 {
      use crate::forces::no_explode::omega_l;

      let close_print = separation_distance < 0.2 * f64::min(r1, r2);

      // let omega_0 = omega_0_from_k(k, m);
      // assert!(omega_0 >= 0.);

      let omega_l = omega_l(k, m, b);
      // println!("omega_l: {}", omega_l);

      // T = 1/f = 2\pi/\omega
      // Time of collision is T/2
      let collision_time = std::f64::consts::PI / omega_l;
      let collision_time_dt = collision_time / self.desired_collision_step_count as f64;
      if close_print { println!("coll_time: {}, dt_a: {}", collision_time, collision_time_dt); }

      // TODO: abstract out gravity forces

      // how far two particles can intersect without things getting out of hand
      let max_ok_pen_estimate = f64::max(r1, r2) * self.spring_derivation.get_pen_fraction();

      if close_print { println!("sep_dist = {:e}, max_pen = {:e}, ratio = {}",separation_distance, max_ok_pen_estimate, separation_distance / max_ok_pen_estimate); }

      if separation_distance < -10.0*max_ok_pen_estimate {
        println!("!!!Bad overlap!!! ratio: {} for {} and {}", separation_distance / max_ok_pen_estimate, i1, i2);
      }

      let (mut dt, distance_for_global_speed_estimate) = if separation_distance <= 0. {
          debugln!("colliding",);
          // colliding

          (collision_time_dt, max_ok_pen_estimate / self.desired_collision_step_count as f64)
      } else {
          // NOTE: this right here injects relative_speed_estimate into the collision time calculation
          let current_impact_speed = current_impact_vel.abs();

          // v * t = d
          let impact_time_dt = separation_distance / current_impact_speed;
          if close_print { println!("impact_time_dt: {} = {:e} / {:e}", impact_time_dt, separation_distance, current_impact_speed); }

          // max( dist/(2*v_normal) and 1/(\omega_0 C) )

          // should this be min?
          // No: otherwise we get a zeno's paradox
          // the collision should be processed at steps of dt
          (
              f64::max(
                  impact_time_dt.abs() / 2.,
                  collision_time_dt, // 1. / (omega_l * self.desired_collision_step_count as f64),
              ),
              f64::max(separation_distance, max_ok_pen_estimate),
          )
      };
      if close_print { println!("dt_b = {}", dt); }

      // impact_time_estimate is to ensure that if there is some fast moving particles around,
      // dt will be small enough such that if one particle in this pair gets hit, this pair will get updated properly
      // this is if one of the fastest particles suddenly crashes into one of the pair of particles
      // processed here and transfers all its speed
      let impact_time_dt_from_speed_estimate =
          distance_for_global_speed_estimate / relative_speed_estimate;
      if close_print { 
        println!("itdfse = {}, dist = {:e}, rel = {:e}", impact_time_dt_from_speed_estimate, distance_for_global_speed_estimate, relative_speed_estimate);
      }

      dt = f64::min(dt, impact_time_dt_from_speed_estimate);
      if close_print { println!("dt_c = {}", dt); }

      if dt <= 0. {
          panic!(
              "dt is 0. This should not happen and will create an infinite loop.\n separation distance: {}\n current_impact_vel: {}\n k: {}\n m: {}\n b: {}\n",
              separation_distance, current_impact_vel, k, m, b
          );
      }

      dt
  }
}

impl<SD: SpringDerivation + Sync + Send> EventForce for GravityAndSoftSphereEventForce<SD> {
  type SingleParticleData = HashMap<(usize, usize), f64>;

  fn get_all_particle_data(&mut self) -> Vec<Self::SingleParticleData> {
    println!("Get SPD: {:?}", self.collision_velocity);
    std::mem::replace(&mut self.collision_velocity, vec![])
  }

  fn set_all_particle_data(&mut self, spds: Vec<Self::SingleParticleData>) {
    self.collision_velocity = spds;
    println!("Set SPD: {:?}", self.collision_velocity);
  }

  fn check_data_for_events(&self, next_time: f64) -> Vec<SingleParticleEvent> {
    // TODO: Change this to run in parallel is a Crossbeam queue for the events.
    let mut events = vec![];
    self.collision_velocity.iter().enumerate().for_each(|t| {
      let (i1, v) = t;
      for ((i2, _), _) in v {
        if let None = self.collision_velocity[*i2].get(&(i1, 0)) {
          println!("No match! {} {}", i1, i2);
          events.push(SingleParticleEvent { event_time: next_time, added_time: next_time, index: *i2 });
        }
      }
    });
    events
  }

  fn particle_particle(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, spd: &mut Self::SingleParticleData, mirror_num: usize, dt: f64) -> (f64, Vector) {
    // Check separattion
    let p2x = p2.x + p2.v * (p1.time - p2.time);
    let dx = p2x - p1.x;
    let dist = dx.mag();
    let dv = p2.v - p1.v;
    let separation_distance = dist - (p1.r + p2.r);
    let close_print = separation_distance < 0.2 * f64::min(p1.r, p2.r);
    let vel = dv.mag();
    let last_impact_vel = *spd.get(&(i2, mirror_num)).unwrap_or(&0.0);
    let impact_vel = f64::max(vel, last_impact_vel);
    if separation_distance < -10.0* p1.r * self.spring_derivation.get_pen_fraction() {
      println!("Bad Data i1={}, i2={}, x1={}, x2={}, dx={}, dist={:e}, sep={:e}", i1, i2, p1.x, p2.x, dx, dist, separation_distance);
    }
    let reduced_mass = (p1.m * p2.m) / (p1.m + p2.m);
    let (b, k) = self.spring_derivation.b_and_k(impact_vel, reduced_mass, f64::max(p1.r, p2.r));
    let relative_speed_estimate = 3.0 * (p1.r + p2.r);
    let impact_time_interval = self.get_next_time(i1, i2, separation_distance, impact_vel, k, reduced_mass, b, relative_speed_estimate, p1.r, p2.r);
    if close_print {
      println!("p:p i1:{} i2:{} dx:{} dist:{:e} dv:{} sep_dist:{:e} vel:{:e} impact_vel: {:e} last_impact_vel:{:e} time_int:{}", i1, i2, dx, dist, dv, separation_distance, vel, impact_vel, last_impact_vel, impact_time_interval);
    }
    if separation_distance > 0.0 {
      // Calculate gravity
      let mag = p2.m / (dist * dist * dist);
      // Clear impact velocity if it exists
      let re = spd.remove_entry(&(i2, mirror_num));
      if let Some(re) = re {
        println!("Particle {} removing the entry {:?} for {}", i1, re, i2);
      }
      // Calculate event time delta
      if close_print { println!("gravity {}", dx*mag); }
      (impact_time_interval, dx * mag)
    } else {
      println!("Colliding, i1:{}, i2:{}, dist = {:e}, k={:e}, b={:e}", i1, i2, dist, k, b);
      // Calculate collision force
      let f_spring = (dx / dist) * k * separation_distance;
      let f_damp = dv * b;
      println!("spring {:e} {:e} {} {} {}", k, b, f_spring, f_damp, (f_spring + f_damp) / p1.m);
      // Set impact velocity
      spd.insert((i2, mirror_num), impact_vel);
      println!("SPD {:?}, {:p}", spd, spd);

      (impact_time_interval, (f_spring + f_damp) / p1.m)
    }
  }

  fn particle_group(&self, i1: usize, p1: &Particle, cm_x: &Vector, cm_m: f64, dt: f64) -> (f64, Vector) {
    let dx = p1.x - *cm_x;
    let dist = dx.mag();
    let mag = p1.m * cm_m / (dist * dist * dist);
    (dt, dx * mag)
  }
}

#[cfg(test)]
mod test {
    use crate::{forces::gravity_and_soft_sphere_event_force::GravityAndSoftSphereEventForce, forces::no_explode::Rotter};

  #[test]
  fn check_next_time() {
    let force = GravityAndSoftSphereEventForce::new(2, Rotter::new(0.5, 0.01), 20);
    
    let k = 1.0;
    let b = 1.0;
    let m = 1.0;

    let expected_coll_time = 3.6275987284684357;
    let expected_colliding_step_time = expected_coll_time / 20.0;

    let check_time_separate = force.get_next_time(0, 1, 1.0, 1.0, k, m, m, 1.0, 1.0, 1.0);
    assert_eq!(check_time_separate, 0.5);

    let check_time_overlap = force.get_next_time(0, 1, -0.001, 1.0, k, m, m, 0.00000001, 1.0, 1.0);
    assert_eq!(check_time_overlap, expected_colliding_step_time);

    let check_time_overlap = force.get_next_time(0, 1, -0.001, 1.0, k, m, m, 1.0, 1.0, 1.0);
    assert_eq!(check_time_overlap, (0.01 / 20.0) * 1.0);
  }

  #[test]
  fn check_particle_particle() {

  }
}