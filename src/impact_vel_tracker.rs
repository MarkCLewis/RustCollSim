use std::{collections::HashMap, hash::Hash, ops::Index};

use crate::particle::ParticleIndex;

#[derive(PartialEq, Eq, Hash, PartialOrd, Ord)]
struct IndexPair(ParticleIndex, ParticleIndex);

impl IndexPair {
    pub fn swap(&self) -> Self {
        Self(self.1, self.0)
    }
}

struct ImpactVelocityTracker {
    // p1 < p2
    data: HashMap<IndexPair, (f64, usize)>,
}

impl ImpactVelocityTracker {
    pub fn add(&mut self, p1: ParticleIndex, p2: ParticleIndex, v: f64, step_count: usize) {
        if p1 < p2 {
            self.data.insert(IndexPair(p1, p2), (v, step_count));
        } else {
            // ignore
        }
    }

    /// tries p1, p2
    pub fn get(&mut self, p1: ParticleIndex, p2: ParticleIndex) -> Option<(f64, usize)> {
        let idx = if p1 < p2 {
            IndexPair(p1, p2)
        } else {
            IndexPair(p2, p1)
        };
        self.data.get(&idx).copied()
    }

    /// deletes all elements where the step < current_step
    pub fn trim(&mut self, current_step: usize) {
        self.data
            .drain_filter(|_, (_, us)| *us < current_step)
            .count();
    }
}
