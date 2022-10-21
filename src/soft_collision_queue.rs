use std::{cell::Cell, cmp::Ordering, collections::BinaryHeap, hash::Hash};

use crate::{
    kd_tree::KDTree,
    particle::{dot, Particle, ParticleIndex},
    util::borrow_two_elements,
};

/**
 * This file contains code elements used for the higher time resolution integration of close interactions.
 */

#[derive(Clone, Copy, Debug)]
struct ForceEvent {
    time: f64,
    create_time: f64, // time event was scheduled, to compute time steps
    p1: ParticleIndex,
    p2: ParticleIndex,
    max_vel: f64,
}
impl PartialEq for ForceEvent {
    fn eq(&self, other: &Self) -> bool {
        self.time.eq(&other.time)
    }
}
// binary heap is max, but we want the closest time event, so we need to reverse the order
impl PartialOrd for ForceEvent {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.time.partial_cmp(&other.time).map(|e| match e {
            Ordering::Less => Ordering::Greater, // flipping order as binary heap is max heap
            Ordering::Greater => Ordering::Less,
            Ordering::Equal => Ordering::Equal,
        })
    }
}
impl Eq for ForceEvent {} // time better not be NaN or infinity, otherwise this breaks
impl Ord for ForceEvent {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).expect(&format!(
            "Got invalid timestamp in ForceEvent: {} and {}",
            self.time, other.time
        ))
    }
}

pub struct ForceQueue {
    queue: BinaryHeap<ForceEvent>,
    big_time_step: f64, // the main time step
}

fn fast_forward(p: &mut Particle, time: f64) {
    todo!();
}

impl ForceQueue {
    /// fill queue, call at the beginning of step
    pub fn find_initial_collisions(&mut self, tree: &Vec<KDTree>, particles: &Vec<Particle>) {
        self.queue.clear();
        // do stuff -> find potential collisions
    }

    pub fn run_through_collision_pairs(&mut self, particles: &mut Vec<Particle>) {
        loop {
            if let Some(event) = self.queue.pop() {
                // do stuff -> process collisions
                let (p1, p2) = borrow_two_elements(particles, event.p1.0, event.p2.0);

                fast_forward(p1, event.time);
                fast_forward(p2, event.time);

                let dt = event.time - event.create_time;
                assert!(dt > 0.);

                // compute local forces?
                //

                // if moving at each other, reschedule at dt/2, but not less than big_time_step/100
                let next_time = event.time + (dt / 2.).max(self.big_time_step / 100.);

                self.queue.push(ForceEvent {
                    time: next_time,
                    create_time: event.time,
                    p1: event.p1,
                    p2: event.p2,
                    max_vel: todo!(),
                });

                println!("{:?}, {:?}", p1, p2);
            } else {
                break;
            }
        }
    }
}

// / idea:
// / step forward to 1/2 of distance, with abs min to avoid zeno's paradox -> min interval - dt/100
// / find timestamp to resolve collision
// / when adding to PQ, update vel
// / update position before or after vel?
// / have set time steps, like a collision should be 5-10 or so
// / pairs keep local vel
// / no bullets are in the rings
// /
