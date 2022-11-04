use std::{cmp::Ordering, collections::BinaryHeap};

use crate::{
    particle::{Particle, ParticleIndex},
    util::borrow_two_elements,
    vectors::Vector,
};

/**
 * This file contains code elements used for the higher time resolution integration of close interactions.
 */

#[derive(Clone, Copy, Debug)]
pub struct ForceEvent {
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
    pub queue: BinaryHeap<ForceEvent>,
    big_time_step: f64, // the main time step
    compute_local_acceleration: F,
    desired_collision_step_count: u32,
    minimum_time_step: f64,
}

mod support {
    pub enum PushPq {
        DoPush,
        NoPush,
    }
}

impl<F: Fn(&mut Particle, &mut Particle) -> ([f64; 3], [f64; 3])> ForceQueue<F> {
    pub fn new(big_time_step: f64, compute_local_acceleration: F) -> Self {
        Self {
            queue: BinaryHeap::new(),
            big_time_step,
            compute_local_acceleration,
            desired_collision_step_count: 10,
            minimum_time_step: big_time_step / 100.,
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

    pub fn do_one_step(&mut self, particles: &mut Vec<Particle>, next_sync_step: f64) {
        //self.queue.clear();

        self.run_through_collision_pairs(particles, next_sync_step);

        Self::end_step(particles, next_sync_step);
    }

    /// fast-forward every particle to the next time step to sync them
    pub fn end_step(particles: &mut Vec<Particle>, next_sync_step: f64) {
        for p in particles {
            Self::fast_forward(p, next_sync_step);
        }
    }

    // the returned time will not be greater than next_sync_step
    fn get_next_time(
        &self,
        separation_distance: f64,
        next_sync_step: f64,
        current_impact_vel: f64,
        event_time: f64,
        m: f64,
        r: f64,
    ) -> (f64, support::PushPq) {
        use crate::no_explode::{omega_0_from_k, rotter::b_and_k};

        let omega_0 = omega_0_from_k(b_and_k(current_impact_vel, m, r).1, m);
        assert!(omega_0 > 0.);

        let dt = f64::max(
            self.minimum_time_step,
            if separation_distance < 0. {
                // colliding
                // 1/(\omega_0 C), => C = step num
                1. / (omega_0 * self.desired_collision_step_count as f64)
            } else {
                // v * t = d
                let impact_time_dt = separation_distance / current_impact_vel;
                // max( dist/(2*v_normal) and 1/(\omega_0 C) )

                f64::max(
                    impact_time_dt.abs() / 2.,
                    1. / (omega_0 * self.desired_collision_step_count as f64),
                )
            },
        );
        let mut next_time = event_time + dt;

        // if moving at each other, reschedule at dt/2, but not less than big_time_step/100
        let repush_to_pq = if next_time > next_sync_step {
            next_time = next_sync_step;
            support::PushPq::NoPush
        } else {
            support::PushPq::DoPush
        };

        (next_time, repush_to_pq)
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

                // compute local forces
                let (acc1, acc2) = (self.compute_local_acceleration)(p1, p2);

                // FIXME: issue: integrating the velocity requires knowing next event time
                // computing next event time requires knowing velocity

                // if not overlapping, we need a new impact velocity
                let updated_impact_vel = if separation_distance < 0. {
                    f64::max(event.impact_vel, current_impact_vel)
                } else {
                    current_impact_vel
                };

                let (next_time, repush_to_pq) = self.get_next_time(
                    separation_distance,
                    next_sync_step,
                    current_impact_vel,
                    event.time,
                    f64::max(p1.m, p2.m),
                    f64::max(p1.r, p2.r),
                );

                p1.v = (Vector(p1.v) + Vector(acc1) * (next_time - event.time)).0;
                p2.v = (Vector(p2.v) + Vector(acc2) * (next_time - event.time)).0;

                match repush_to_pq {
                    support::PushPq::DoPush => {
                        self.queue.push(ForceEvent {
                            time: next_time,
                            last_time: event.time,
                            p1: event.p1,
                            p2: event.p2,
                            impact_vel: updated_impact_vel,
                        });
                    }
                    support::PushPq::NoPush => {
                        // do nothing
                    }
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
