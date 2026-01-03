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

use crate::{design::system::{Force, Particle, Population}, util::parallel_subset_process::{parallel_subset_process_recur, parallel_subset_process_recur_mut, parallel_subset_process_recur_mut_res, parallel_subset_process_recur_mut1, parallel_subset_process_recur_mut1_data_res, parallel_subset_process_recur_mut2, parallel_subset_process_recur_mut2_res}, vectors::Vector};
use rayon::{prelude::*};

pub trait Traverser: Sync {
  fn setup(&mut self, pop: &impl Population);
  // fn all_pairs<F: EventForce>(&self, force: &F);
  fn for_one<F: EventForce>(&self, i1: usize, p1: &Particle, spd1: &mut F::SingleParticleData, force: &F, dt: f64, pop: &impl Population) -> (f64, Vector);
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
  fn particle_particle(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, spd: &mut Self::SingleParticleData, mirror_num: usize, dt: f64) -> (f64, Vector);
  fn particle_group(&self, i1: usize, p1: &Particle, cm_x: &Vector, cm_m: f64, dt: f64) -> (f64, Vector);
}

pub struct SingleParticleEventForcing<T: Traverser, F: EventForce, Q: EventQueue> {
  traverser: T,
  event_force: F,
  queue: Q,
  dt: f64,
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
    let (delta_vs, events): (Vec<_>, Vec<_>) = pop.particles().par_iter().zip(spds.par_iter_mut()).enumerate().map(|t| {
      let (i1, (p1, spd)) = t;
      println!("Initail SPD: {:?}, {:p}", spd, spd);
      let (event_time, delta_v) = self.traverser.for_one(i1, p1, spd, &self.event_force, self.dt, pop);
      println!("Final SPD: {:?}, {:p}", spd, spd);
      println!("first kick {} {} {}", i1, delta_v, p1.v);
      (delta_v, if event_time < self.dt { Some(SingleParticleEvent::new(event_time, 0.0, i1))} else { None })
    }).unzip();
    delta_vs.par_iter().zip(pop.particles_mut().par_iter_mut()).for_each(|t| {
      let (dv, p) = t;
      p.kick(dv);
    });
    // println!("Enqueue {} events", events.len());
    self.queue.enqueue_many(events.into_iter().flatten());
    // println!("pop size {}, spds size {}", pop.particles().len(), spds.len());
    self.event_force.set_all_particle_data(spds);
    while !self.queue.is_empty() {
      let batch = self.queue.get_next_batch();
      println!("Batch size: {}", batch.len());
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
      let mut times_and_deltas = vec![(0.0, Vector::new(0.0, 0.0, 0.0)); elements.len()];
      parallel_subset_process_recur_mut_res(pop.particles(), &mut spds[..], &elements[..], &mut times_and_deltas[..], &|index, p, spd| {
        println!("Before SPD: {:?} {:p}", spd, spd);
        let ret = self.traverser.for_one(index, p, spd, &self.event_force, self.dt, pop);
        println!("After SPD: {:?} {:p}", spd, spd);
        ret
      });
      self.event_force.set_all_particle_data(spds);
      let mut new_events: Vec<Option<SingleParticleEvent>> = vec![None; elements.len()];
      parallel_subset_process_recur_mut1_data_res(pop.particles_mut(), &elements[..], &times_and_deltas[..], &mut new_events[..], &|index, p, td| {
        let (delta_t, delta_v) = td;
        p.kick(&delta_v);
        println!("post-kick {} {} {} {} {:e}", index, delta_v, p.v, common_time, delta_t);
        assert_eq!(common_time, p.time);
        assert!(*delta_t > 0.0);
        if *delta_t + common_time < self.dt - 1e-8 * self.dt {  // Account for epsilon in float error.
          Some(SingleParticleEvent::new(common_time + *delta_t, common_time, index))
        } else {
          None
        }
      });
      self.queue.enqueue_many(new_events.into_iter().flatten());
      let next_time = self.queue.next_time();
      if let Some(next_time) = next_time {
        self.queue.enqueue_many(self.event_force.check_data_for_events(next_time));
      }
    }
  }
}