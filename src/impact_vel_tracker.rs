use std::collections::HashMap;

use crate::particle::ParticleIndex;

// #[derive(PartialEq, Eq, Hash, PartialOrd, Ord)]
// struct IndexPair(ParticleIndex, ParticleIndex);

// impl IndexPair {
//     pub fn swap(&self) -> Self {
//         Self(self.1, self.0)
//     }
// }

pub fn smaller_first(p1: ParticleIndex, p2: ParticleIndex) -> (ParticleIndex, ParticleIndex) {
    if p1 < p2 {
        (p1, p2)
    } else {
        (p2, p1)
    }
}

pub struct ImpactVelocityTracker {
    // p1 < p2
    data: HashMap<(ParticleIndex, ParticleIndex), (f64, usize)>,

    #[cfg(test)]
    collision_step_count: HashMap<(ParticleIndex, ParticleIndex), usize>,
}

impl ImpactVelocityTracker {
    pub fn new() -> Self {
        Self {
            data: HashMap::new(),
            #[cfg(test)]
            collision_step_count: HashMap::new(),
        }
    }

    #[cfg(test)]
    pub fn get_updated_count(&self, p1: ParticleIndex, p2: ParticleIndex) -> Option<usize> {
        self.collision_step_count
            .get(&smaller_first(p1, p2))
            .copied()
    }

    pub fn add(&mut self, p1: ParticleIndex, p2: ParticleIndex, v: f64, step_count: usize) {
        #[cfg(test)]
        {
            // track how many times a pair has had its impact vel updated, useful for determining
            // how many steps a collision took
            let times_updated = self
                .collision_step_count
                .get(&smaller_first(p1, p2))
                .unwrap_or(&0);
            self.collision_step_count
                .insert(smaller_first(p1, p2), times_updated + 1);
        }

        self.data.insert(smaller_first(p1, p2), (v, step_count));
    }

    /// tries p1, p2
    pub fn get(&self, p1: ParticleIndex, p2: ParticleIndex) -> Option<(f64, usize)> {
        self.data.get(&smaller_first(p1, p2)).copied()
    }

    /// deletes all elements where the step < current_step
    pub fn trim(&mut self, current_step: usize) {
        self.data
            .drain_filter(|_, (_, us)| *us < current_step)
            .count();
    }
}
