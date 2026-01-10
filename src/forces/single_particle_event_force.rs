// Generic event-based force.

// Things that I want to be able to vary.
// - Traveral approach
// - Force calculation
// - Queueing of single particles or pairs (events)

// Can I do a generic event force?
// It takes a traverser, a force function (possibly two), an event time function, and a queue.
// Traverse all pairs initially to get forces and add events to the queue.
// Then process the queue

use std::{cmp::Ordering, fmt::Debug};

use crate::{design::system::{Force, Particle, Population}, util::parallel_subset_process::{parallel_subset_process_recur_mut_res, parallel_subset_process_recur_mut1_data, parallel_subset_process_recur_mut1_data_res}, vectors::Vector};
use rayon::{prelude::*};

pub trait Traverser: Sync {
  fn setup(&mut self, pop: &impl Population);
  fn accel_for_one<F: EventForce>(&self, i1: usize, p1: &Particle, spd1: &mut F::SingleParticleData, force: &F, pop: &impl Population) -> Vector;
  fn time_step_for_one<F: EventForce>(&self, i1: usize, p1: &Particle, spd1: &F::SingleParticleData, force: &F, pop: &impl Population, accs: &Vec<Vector>) -> f64;
}

#[derive(Debug, PartialEq, Clone, Default)]
pub struct SingleParticleEvent {
  pub event_time: f64,
  pub added_time: f64,
  pub index: usize,
}

impl SingleParticleEvent {
  pub fn new(event_time: f64, added_time: f64, index: usize) -> SingleParticleEvent {
    assert!(event_time > added_time);
    SingleParticleEvent { event_time, added_time, index }
  }
  pub fn half_step_time(&self) -> f64 {
    0.5 * (self.event_time + self.added_time)
  }
}

impl Eq for SingleParticleEvent {}

impl Ord for SingleParticleEvent {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.event_time.total_cmp(&self.event_time)
            .then(other.added_time.total_cmp(&self.added_time))
    }
}

impl PartialOrd for SingleParticleEvent {
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub trait EventQueue {
  fn enqueue_one(&mut self, e: SingleParticleEvent);
  fn enqueue_many(&mut self, items: impl IntoIterator<Item = SingleParticleEvent>);
  fn enqueue_many_optional(&mut self, items: impl IntoIterator<Item = Option<SingleParticleEvent>>);
  fn process_next(&mut self, f: impl FnMut(&SingleParticleEvent) -> Option<SingleParticleEvent>);
  fn process_batch(&mut self, f: impl FnMut(&SingleParticleEvent, f64) -> Option<SingleParticleEvent>);
  fn get_next_batch(&mut self) -> Vec<SingleParticleEvent>;
  fn next_time(&self) -> Option<f64>;
  fn is_empty(&self) -> bool;
}

pub trait EventForce: Sync {
  type SingleParticleData: Sync + Send + Default + Debug;
  fn get_all_particle_data(&mut self) -> Vec<Self::SingleParticleData>;
  fn set_all_particle_data(&mut self, spds: Vec<Self::SingleParticleData>);
  fn check_data_for_events(&self, next_time: f64) -> Vec<SingleParticleEvent>;
  fn particle_particle_accel(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, spd: &mut Self::SingleParticleData, mirror_num: usize) -> Vector;
  fn particle_particle_time_step(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, spd: &Self::SingleParticleData, accels: &Vec<Vector>, mirror_num: usize) -> f64;
  fn particle_group_accel(&self, i1: usize, p1: &Particle, cm_x: &Vector, cm_m: f64, dt: f64) -> Vector;
}

pub struct SingleParticleEventForcing<T: Traverser, F: EventForce, Q: EventQueue> {
  traverser: T,
  event_force: F,
  queue: Q,
  dt: f64,
}

fn check_time_step(mut time_step: f64, current_time: f64, dt: f64) -> f64 {
  println!("Check time: {} {} {}", time_step, current_time, dt);
  if time_step + current_time > dt {
    time_step = dt - current_time;
  }
  if time_step < 1e-12 * dt {
    println!("Warning! Changing step size from {:e} to {:e}", time_step, 1e-12 * dt);
    time_step = 1e-12 * dt;
  }
  time_step
}

impl<T: Traverser, F: EventForce, Q: EventQueue> SingleParticleEventForcing<T, F, Q> {
  pub fn new<TA: Traverser, FA: EventForce, QA: EventQueue>(t: TA, f: FA, q: QA, dt: f64) -> SingleParticleEventForcing<TA, FA, QA> {
    SingleParticleEventForcing { traverser: t, event_force: f, queue: q, dt: dt }
  }
}

impl<T: Traverser, F: EventForce, Q: EventQueue> Force for SingleParticleEventForcing<T, F, Q> {
  fn apply_force(&mut self, pop: &mut impl Population) {
    self.traverser.setup(pop);
    let mut spds = self.event_force.get_all_particle_data();
    // println!("pop size {}, spds size {}", pop.particles().len(), spds.len());
    let mut accels: Vec<Vector> = pop.particles().par_iter().zip(spds.par_iter_mut()).enumerate().map(|t| {
      let (i1, (p1, spd)) = t;
      self.traverser.accel_for_one(i1, p1, spd, &self.event_force, pop)
    }).collect();
    let (delta_vs, events): (Vec<Vector>, Vec<Option<SingleParticleEvent>>) = pop.particles().par_iter().zip(spds.par_iter()).enumerate().map(|t| {
      let (i1, (p1, spd)) = t;
      let time_step = self.traverser.time_step_for_one(i1, p1, spd, &self.event_force, pop, &accels);
      let time_step = check_time_step(time_step, 0.0, self.dt);
      // println!("first time i1:{} {} {} {}", i1, accels[i1], p1.v, time_step);
      (accels[i1] * time_step, if time_step < self.dt { Some(SingleParticleEvent::new(time_step, 0.0, i1))} else { None })
    }).unzip();
    pop.particles_mut().par_iter_mut().zip(delta_vs.par_iter()).for_each(|t| {
      let (p1, dv) = t;
      p1.kick(&dv);
    });
    // println!("Enqueue {} events", events.len());
    self.queue.enqueue_many_optional(events);
    // println!("pop size {}, spds size {}", pop.particles().len(), spds.len());
    self.event_force.set_all_particle_data(spds);
    while !self.queue.is_empty() {
      let batch = self.queue.get_next_batch();
      println!("Batch: {}, {:?}", batch.len(), batch);
      let common_time = batch.first().map(|spe| spe.event_time ).unwrap_or(self.dt);
      println!("Common time: {}", common_time);
      let mut elements: Vec<usize> = batch.into_iter().map(|spe| spe.index ).collect();
      elements.sort();
      elements.dedup();
      pop.particles_mut().par_iter_mut().for_each(|p| {
        p.advance_to(common_time);
        assert_eq!(common_time, p.time);
      });
      let mut spds = self.event_force.get_all_particle_data();
      let mut accels_small = vec![Vector::new(0.0, 0.0, 0.0); elements.len()]; // TODO: Needs to be full length of particles for indexing.
      parallel_subset_process_recur_mut_res(pop.particles(), &mut spds[..], &elements[..], &mut accels_small[..], &|index, p, spd| {
        self.traverser.accel_for_one(index, p, spd, &self.event_force, pop)
      });
      for i in 0..elements.len() {
        accels[elements[i]] = accels_small[i];
      }
      let mut dvs_and_events: Vec<(Vector, Option<SingleParticleEvent>)> = vec![(Vector::new(0.0, 0.0, 0.0), None); elements.len()];
      // parallel_subset_process_recur_mut1_data_res(pop.particles_mut(), &elements[..], &accels[..], &mut new_events[..], &|index, p, td| {
      //   let (delta_t, delta_v) = td;
      parallel_subset_process_recur_mut_res(pop.particles(), &mut spds[..], &elements[..], &mut dvs_and_events[..], &|index, p, spd| {
        let time_step = self.traverser.time_step_for_one(index, p, spd, &self.event_force, pop, &accels);
        let time_step = check_time_step(time_step, p.time, self.dt);
        let dv = accels[index] * time_step;
        // p.kick(&dv);
        // println!("post-kick {} {} {} {} {:e}", index, dv, p.v, common_time, time_step);
        assert_eq!(common_time, p.time);
        assert!(time_step > 0.0);
        (dv, if time_step + common_time < self.dt - 1e-12 * self.dt {  // Account for epsilon in float error.
          Some(SingleParticleEvent::new(common_time + time_step, common_time, index))
        } else {
          None
        })
      });
      self.event_force.set_all_particle_data(spds);
      parallel_subset_process_recur_mut1_data(pop.particles_mut(), &elements[..], &dvs_and_events[..], &|index, p, td| {
        let (delta_v,_) = td;
        p.kick(&delta_v);
        // println!("post-kick {} {} {}", index, delta_v, p.v);
      });
      self.queue.enqueue_many_optional(dvs_and_events.into_iter().map(|t| {
        let (_, event) = t;
        event
      }));
      let next_time = self.queue.next_time();
      if let Some(next_time) = next_time {
        self.queue.enqueue_many(self.event_force.check_data_for_events(next_time));
      }
    }
  }
}

#[cfg(test)]
mod test {
  use crate::{boundary_conditions::{open_boundaries::OpenBoundary}, design::{basic_population::BasicPopulation, system::{Force, Particle, Population}}, forces::{brute_force_particle_traversal::BruteForceParticleTraversal, gravity_and_soft_sphere_event_force::GravityAndSoftSphereEventForce, heap_pq::HeapPQ, no_explode::Rotter, single_particle_event_force::{SingleParticleEventForcing}}, vectors::Vector};

  // TODO: wrap this in a loop that goes through different parameters
  #[test]
  fn coef_rest_and_overlap_tests() {
    let dt = 0.1;
    let particles = vec![
      Particle::new(
        Vector::new(-2.0, 0.0, 0.0), 
        Vector::new(1.0, 0.0, 0.0),
         1.0, 1.0, 0.0),
      Particle::new(
        Vector::new(2.0, 0.0, 0.0), 
        Vector::new(-1.0, 0.0, 0.0),
         1.0, 1.0, 0.0)
    ];
    let boundary_conditions = OpenBoundary {};
    let mut pop = BasicPopulation::new(particles, boundary_conditions);
    type Trav<'a> = BruteForceParticleTraversal;
    let traverser = BruteForceParticleTraversal::new();
    type GravEventForce = GravityAndSoftSphereEventForce<Rotter>;
    let spring = Rotter::new(0.5, 0.02);
    let event_force = GravityAndSoftSphereEventForce::new(2, spring, 20);
    let queue = HeapPQ::new();
    let mut grav_coll_force = SingleParticleEventForcing::<Trav<'_>, GravEventForce, HeapPQ>::new(traverser, event_force, queue, dt);

    let mut done = false;
    let mut last_vel = 0.0;
    let mut vel = 0.0;
    let mut step = 0;
    while !done {
      grav_coll_force.apply_force(&mut pop);
      pop.end_step(dt);
      let parts = pop.particles();
      let dx = parts[0].x - parts[1].x;
      let dist = dx.mag();
      let dv = parts[0].v - parts[1].v;
      println!("step: {} dx: {}, dv: {}", step, dx, dv);
      vel = dv.mag();
      if dist > parts[0].r + parts[1].r {
        if dv.dot(&dx) > 0.0 {
          done = true;
        } else {
          last_vel = vel;
        }
      }
      step += 1;
      if step > 50 {
        done = true;
      }
    }
    assert!(vel > last_vel * 0.48 && vel < last_vel * 0.52);
    
  }

  #[test]
  fn check_particle_particle() {

  }
}