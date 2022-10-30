use std::{cmp::Ordering, collections::BinaryHeap};

use crate::{
    kd_tree::KDTree,
    particle::{Particle, ParticleIndex},
    util::borrow_two_elements,
    vectors::Vector,
};

/**
 * This file contains code elements used for the higher time resolution integration of close interactions.
 */

#[derive(Clone, Copy, Debug)]
struct ForceEvent {
    time: f64,
    last_time: f64,
    p1: ParticleIndex,
    p2: ParticleIndex,
    impact_vel: f64,
}
// data like impact vel needs to be kept from one timestep to the next

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

pub struct ForceQueue<F: Fn(&mut Particle, &mut Particle) -> ([f64; 3], [f64; 3])> {
    queue: BinaryHeap<ForceEvent>,
    big_time_step: f64, // the main time step
    compute_local_acceleration: F,
}

impl<F: Fn(&mut Particle, &mut Particle) -> ([f64; 3], [f64; 3])> ForceQueue<F> {
    pub fn new(big_time_step: f64, compute_local_acceleration: F) -> Self {
        Self {
            queue: BinaryHeap::new(),
            big_time_step,
            compute_local_acceleration,
        }
    }

    fn fast_forward(particle: &mut Particle, current_time: f64) {
        // kick-step
        let dt = current_time - particle.t;
        assert!(dt > 0.);
        particle.p[0] += particle.v[0] * dt;
        particle.p[1] += particle.v[1] * dt;
        particle.p[2] += particle.v[2] * dt;
        particle.t = current_time;
    }

    pub fn do_one_step(
        &mut self,
        particles: &mut Vec<Particle>,
        tree: &Vec<KDTree>,
        next_sync_step: f64,
    ) {
        self.find_initial_collisions(tree, particles);

        self.run_through_collision_pairs(particles, next_sync_step);

        Self::end_step(particles, next_sync_step);
    }

    /// fast-forward every particle to the next time step to sync them
    pub fn end_step(particles: &mut Vec<Particle>, next_sync_step: f64) {
        for p in particles {
            Self::fast_forward(p, next_sync_step);
        }
    }

    /// fill queue, call at the beginning of step
    pub fn find_initial_collisions(&mut self, tree: &Vec<KDTree>, particles: &Vec<Particle>) {
        self.queue.clear();
        // do stuff -> find potential collisions -> other particles within a dt*v range
    }

    pub fn run_through_collision_pairs(
        &mut self,
        particles: &mut Vec<Particle>,
        next_sync_step: f64,
    ) {
        loop {
            if let Some(event) = self.queue.pop() {
                // do stuff -> process collisions
                let (p1, p2) = borrow_two_elements(particles, event.p1.0, event.p2.0);

                Self::fast_forward(p1, event.time);
                Self::fast_forward(p2, event.time);

                let unit_to_p2 = (Vector(p2.p) - Vector(p1.p)).unit_vector(); // unit vec from p1 pointing at p2

                // (vel of p2 rel to p1) dot (unit vector pointing at p2 from p1)
                let current_impact_vel = (Vector(p2.v) - Vector(p1.v)) * unit_to_p2;

                let separation_distance = (Vector(p1.p) - Vector(p2.p)).mag() - (p1.r + p2.r);

                let last_time_step = event.time - event.last_time;
                assert!(last_time_step > 0.);

                let mut next_time = if separation_distance < 0. {
                    f64::max(last_time_step / 2., self.big_time_step / 100.)
                } else {
                    // v * t = d
                    let impact_time_dt = separation_distance / current_impact_vel;
                    if impact_time_dt > 0. {
                        // collision is yet to happen
                        event.time + f64::max(impact_time_dt / 2., self.big_time_step / 100.)
                    } else {
                        // collision is in the past, i.e. drifting away from each other
                        f64::max(impact_time_dt * 2., self.big_time_step / 100.)
                    }
                };

                // if moving at each other, reschedule at dt/2, but not less than big_time_step/100
                let repush_to_pq = if next_time > next_sync_step {
                    next_time = next_sync_step;
                    false
                } else {
                    true
                };

                // compute local forces
                let (acc1, acc2) = (self.compute_local_acceleration)(p1, p2);

                p1.v = (Vector(p1.v) + Vector(acc1) * (next_time - event.time)).0;
                p2.v = (Vector(p2.v) + Vector(acc2) * (next_time - event.time)).0;

                // if not overlapping, we need a new impact velocity
                let updated_impact_vel = if separation_distance < 0. {
                    f64::max(event.impact_vel, current_impact_vel)
                } else {
                    current_impact_vel
                };

                if repush_to_pq {
                    self.queue.push(ForceEvent {
                        time: next_time,
                        last_time: event.time,
                        p1: event.p1,
                        p2: event.p2,
                        impact_vel: updated_impact_vel,
                    });
                }

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

// one cycle
// compute global forces (gravity)
// set acceleration of particles
// find possible collisions, add each to queue with some t before the expected collision
// go through collisions, closest first
// fast forward each to time t of the event
// re-compute potential collision and push back to queue (if event past next time step, don't add and deal with next step?)
// at end of timestep, fast-forward all to be in sync
// loop
