use crate::design::event_force::{EventQueue, SingleParticleEvent};

pub struct HeapPQ {

}

impl HeapPQ {
  pub fn new() -> HeapPQ {
    HeapPQ {}
  }
}

impl EventQueue for HeapPQ {
  fn enqueue_one(&mut self, e: SingleParticleEvent) {
    // TODO: Not used yet.
  }

  fn enqueue_many(&mut self, items: impl IntoIterator<Item = SingleParticleEvent>) {

  }

  fn process_next(&mut self, f: impl FnMut(&SingleParticleEvent) -> Option<SingleParticleEvent>) {
    // TODO: Not used yet.
  }

  fn process_batch(&mut self, f: impl FnMut(&SingleParticleEvent, f64) -> Option<SingleParticleEvent>) {
    // TODO: Not used yet.
  }

  fn is_empty(&self) -> bool {
    true
  }
}