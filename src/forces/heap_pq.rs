use std::collections::BinaryHeap;

use rayon::iter::{IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelBridge, ParallelIterator};

use crate::forces::single_particle_event_force::{EventQueue, SingleParticleEvent};

pub struct HeapPQ {
  heap: BinaryHeap<SingleParticleEvent>,
}

impl HeapPQ {
  pub fn new() -> HeapPQ {
    HeapPQ { heap: BinaryHeap::new() }
  }
}

impl EventQueue for HeapPQ {
  fn enqueue_one(&mut self, e: SingleParticleEvent) {
    // TODO: Not used yet.
  }

  fn enqueue_many(&mut self, items: impl IntoIterator<Item = SingleParticleEvent>) {
    for item in items {
      self.heap.push(item);
    }
  }

  fn process_next(&mut self, f: impl FnMut(&SingleParticleEvent) -> Option<SingleParticleEvent>) {
    // TODO: Not used yet.
  }

  fn process_batch(&mut self, mut f: impl FnMut(&SingleParticleEvent, f64) -> Option<SingleParticleEvent>) {
    let first_time = self.heap.peek().map(|spe| {spe.event_time}).unwrap_or(0.0);
    let mut batch = vec![];
    while let Some(spe) = self.heap.pop() && spe.half_step_time() < first_time {
      batch.push(spe);
    }
    // let mut batch: Vec<_> = self.heap.iter().take_while(|spe| { spe.half_step_time() < first_time }).collect();
    batch.iter().map( |spe| {
      f(spe, first_time)
    }).flatten().for_each(|event| { self.heap.push(event)});
  }

  fn next_time(&self) -> Option<f64> {
    self.heap.peek().map(|spe| {spe.event_time})
  }

  fn is_empty(&self) -> bool {
    self.heap.is_empty()
  }

  fn get_next_batch(&mut self) -> Vec<SingleParticleEvent> {
    let first_time = self.heap.peek().map(|spe| {spe.event_time}).unwrap_or(0.0);
    println!("first_event = {:?}", self.heap.peek().unwrap());
    println!("half_step_time = {}", self.heap.peek().unwrap().half_step_time());
    let mut batch = vec![];
    while let Some(spe) = self.heap.pop() && spe.half_step_time() <= first_time {
      batch.push(spe);
    };
    batch
  }
}