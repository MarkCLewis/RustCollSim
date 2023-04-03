use std::collections::HashMap;

use crate::particle::ParticleIndex;

pub fn smaller_first(p1: ParticleIndex, p2: ParticleIndex) -> (ParticleIndex, ParticleIndex) {
    if p1 < p2 {
        (p1, p2)
    } else {
        (p2, p1)
    }
}

#[cfg(test)]
mod collision_analysis {
    #[derive(Debug, Clone, Copy)]
    pub struct ImpactExitVelocityTracking {
        impact_vel: f64,
        exit_vel: f64,
        max_penetration_depth: f64,
    }

    impl ImpactExitVelocityTracking {
        pub fn new(first_velocity: f64, penetration_depth: f64) -> Self {
            Self {
                impact_vel: first_velocity,
                exit_vel: first_velocity,
                max_penetration_depth: penetration_depth,
            }
        }

        /// if we keep overwriting the exit vel while there is a collision, the final
        /// value will be the exit vel (the write just before the collision ends)
        pub fn update(&mut self, velocity: f64, penetration_depth: f64) {
            self.exit_vel = velocity;
            if penetration_depth > self.max_penetration_depth {
                self.max_penetration_depth = penetration_depth;
            }
        }

        /// returns impact vel, exit vel
        pub fn get_data(&self) -> (f64, f64, f64) {
            (self.impact_vel, self.exit_vel, self.max_penetration_depth)
        }
    }
}

pub struct ImpactVelocityTracker {
    // p1 < p2
    data: HashMap<(ParticleIndex, ParticleIndex), (f64, usize)>,

    #[cfg(test)]
    collision_step_count: HashMap<(ParticleIndex, ParticleIndex), usize>,

    #[cfg(test)]
    velocity_analysis:
        HashMap<(ParticleIndex, ParticleIndex), collision_analysis::ImpactExitVelocityTracking>,
}

#[cfg(test)]
impl ImpactVelocityTracker {
    pub fn get_updated_count(&self, p1: ParticleIndex, p2: ParticleIndex) -> Option<usize> {
        self.collision_step_count
            .get(&smaller_first(p1, p2))
            .copied()
    }

    /// (impact, exit)
    pub fn get_analysis(&self, p1: ParticleIndex, p2: ParticleIndex) -> Option<(f64, f64, f64)> {
        self.velocity_analysis
            .get(&smaller_first(p1, p2))
            .map(|entry| entry.get_data())
    }

    /// this should get called every time a pair is processed while colliding
    pub fn velocity_analysis_update(
        &mut self,
        p1: ParticleIndex,
        p2: ParticleIndex,
        current_speed: f64,
        penetration_depth: f64,
    ) {
        let key = &smaller_first(p1, p2);

        // every time this triggers
        match self.velocity_analysis.get_mut(key) {
            None => {
                // first time
                let entry = collision_analysis::ImpactExitVelocityTracking::new(
                    current_speed,
                    penetration_depth,
                );
                self.velocity_analysis.insert(*key, entry);
            }
            Some(entry) => {
                entry.update(current_speed, penetration_depth);
            }
        }
    }
}

impl ImpactVelocityTracker {
    pub fn new() -> Self {
        Self {
            data: HashMap::new(),
            #[cfg(test)]
            collision_step_count: HashMap::new(),
            #[cfg(test)]
            velocity_analysis: HashMap::new(),
        }
    }

    /// v is not current speed, but the impact velocity of the current velocity
    pub fn add(
        &mut self,
        p1: ParticleIndex,
        p2: ParticleIndex,
        v: f64,
        step_count: usize,
        #[cfg(test)] is_colliding: bool,
        #[cfg(test)] penetration_depth: f64,
        #[cfg(test)] current_speed: f64,
    ) {
        #[cfg(test)]
        {
            if is_colliding {
                // track how many times a pair has had its impact vel updated, useful for determining
                // how many steps a collision took
                let times_updated = self
                    .collision_step_count
                    .get(&smaller_first(p1, p2))
                    .unwrap_or(&0);

                self.collision_step_count
                    .insert(smaller_first(p1, p2), times_updated + 1);

                // track the impact and exit velocities of a collision
                self.velocity_analysis_update(p1, p2, current_speed, penetration_depth);
            }
        }

        self.data.insert(smaller_first(p1, p2), (v, step_count));
    }

    /// tries p1, p2
    pub fn get(&self, p1: ParticleIndex, p2: ParticleIndex) -> Option<(f64, usize)> {
        self.data.get(&smaller_first(p1, p2)).copied()
    }

    /// remove velocity entry, if it exists
    pub fn remove(&mut self, p1: ParticleIndex, p2: ParticleIndex) -> Option<(f64, usize)> {
        self.data.remove(&smaller_first(p1, p2))
    }

    /// deletes all elements where the step < current_step
    pub fn trim(&mut self, current_step: usize) {
        self.data
            .drain_filter(|_, (_, us)| *us < current_step)
            .count();
    }
}
