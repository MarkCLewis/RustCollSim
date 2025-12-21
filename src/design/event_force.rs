// Generic event-based force.

// Things that I want to be able to vary.
// - Traveral approach
// - Force calculation
// - Queueing of single particles or pairs (events)

// Can I do a generic event force?
// It takes a traverser, a force function (possibly two), an event time function, and a queue.
// Traverse all pairs initially to get forces and add events to the queue.
// Then process the queue

use crate::{design::system::{Force, Particle}, vectors::Vector};
use rayon::{prelude::*};

pub trait Traverser: Sync {
  fn setup(&mut self, pop: &[Particle]);
  // fn all_pairs<F: EventForce>(&self, force: &F);
  fn for_one<F: EventForce>(&self, i1: usize, p1: &Particle, spd1: &mut F::SingleParticleData, force: &F, dt: f64, pop: &[Particle]) -> (f64, Vector);
}

pub struct SingleParticleEvent {
  index: usize,
  time: f64,
}

pub trait EventQueue {
  fn enqueue_one(&mut self, e: SingleParticleEvent);
  fn enqueue_many(&mut self, items: impl IntoIterator<Item = SingleParticleEvent>);
  fn process_next(&mut self, f: impl FnMut(&SingleParticleEvent) -> Option<SingleParticleEvent>);
  fn process_batch(&mut self, f: impl FnMut(&SingleParticleEvent, f64) -> Option<SingleParticleEvent>);
  fn is_empty(&self) -> bool;
}

pub trait EventForce: Sync {
  type SingleParticleData: Sync + Send;
  fn get_all_particle_data(&mut self) -> Vec<Self::SingleParticleData>;
  fn set_all_particle_data(&mut self, spds: Vec<Self::SingleParticleData>);
  fn get_one_particle_data(&mut self, index: usize) -> Self::SingleParticleData;
  fn set_one_particle_data(&mut self, index: usize, spd: Self::SingleParticleData);
  /// Calculates the force between two particles.
  fn particle_particle(&self, i1: usize, p1: &Particle, i2: usize, p2: &Particle, spd: &mut Self::SingleParticleData, dt: f64) -> (f64, Vector);
  fn particle_group(&self, i1: usize, p1: &Particle, g: &Vector, m: f64, dt: f64) -> (f64, Vector);
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
  fn apply_force(&mut self, pop: &mut [Particle]) {
    self.traverser.setup(pop);
    {
      let mut spds = self.event_force.get_all_particle_data();
      let (delta_vs, events): (Vec<_>, Vec<_>) = pop.par_iter().zip(spds.par_iter_mut()).enumerate().map(|t| {
        let (i1, (p1, spd)) = t;
        let (time, delta_v) = self.traverser.for_one(i1, p1, spd, &self.event_force, self.dt, pop);
        (delta_v, SingleParticleEvent { index: i1, time: time })
      }).unzip();
      delta_vs.par_iter().zip(pop.par_iter_mut()).for_each(|t| {
        let (dv, p) = t;
        p.v += *dv;
      });
      self.queue.enqueue_many(events);
      self.event_force.set_all_particle_data(spds);
    }
    while !self.queue.is_empty() {
      self.queue.process_batch(|spe: &SingleParticleEvent, common_time: f64| {
        let p = &mut pop[spe.index];
        p.advance(common_time - p.time);
        let p = &pop[spe.index];
          let mut spd = self.event_force.get_one_particle_data(spe.index);
          let (time, delta_v) = self.traverser.for_one(spe.index, p, &mut spd, &self.event_force, self.dt, pop);
          self.event_force.set_one_particle_data(spe.index, spd);
          let p = &mut pop[spe.index];
          p.kick(&delta_v);
          if time + p.time <= self.dt {
            Some(SingleParticleEvent { index: spe.index, time: common_time + time })
          } else {
            None
          }
      });
      pop.par_iter_mut().for_each(|p| {
        p.finish_step(self.dt);
      });
    }
  }
}