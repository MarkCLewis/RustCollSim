use std::{cell::RefCell, cmp::Ordering, collections::BinaryHeap};

use crate::{
    debugln,
    impact_vel_tracker::ImpactVelocityTracker,
    particle::{calc_pp_accel, Particle, ParticleIndex},
    util::borrow_two_elements,
    vectors::Vector,
};

/**
 * This file contains code elements used for the higher time resolution integration of close interactions.
 */

#[derive(Clone, Copy, Debug)]
pub struct ForceEvent {
    pub time: f64,
    pub last_time: f64,
    pub p1: ParticleIndex,
    pub p2: ParticleIndex,
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

pub struct SoftSphereForce {
    pub queue: BinaryHeap<ForceEvent>,
    desired_collision_step_count: u32,
    minimum_time_step: f64, // print warn if dt < this
    impact_vel: RefCell<ImpactVelocityTracker>,
}

#[derive(Debug)]
pub enum PushPq {
    DoPush,
    NoPush,
}

struct EventData {
    impact_speed: f64,
    b: f64,
    k: f64,
    separation_distance: f64,
    reduced_mass: f64,
}

impl SoftSphereForce {
    pub fn new(big_time_step: f64) -> Self {
        Self {
            queue: BinaryHeap::new(),
            desired_collision_step_count: 10,
            minimum_time_step: big_time_step / 100.,
            impact_vel: RefCell::new(ImpactVelocityTracker::new()),
        }
    }

    pub fn trim_impact_vel_tracker(&mut self, current_step: usize) {
        self.impact_vel.borrow_mut().trim(current_step);
    }

    fn fast_forward(particle: &mut Particle, current_time: f64) {
        // kick-step
        let dt = current_time - particle.t;
        assert!(
            dt >= 0.,
            "current_time = {}, particle.t = {}",
            current_time,
            particle.t
        );
        particle.p[0] += particle.v[0] * dt;
        particle.p[1] += particle.v[1] * dt;
        particle.p[2] += particle.v[2] * dt;
        particle.t = current_time;
    }

    pub fn do_one_step(
        &mut self,
        particles: &mut Vec<Particle>,
        next_sync_step: f64,
        step_count: usize,
    ) {
        self.run_through_collision_pairs(particles, next_sync_step, step_count);

        Self::end_step(particles, next_sync_step);
    }

    /// fast-forward every particle to the next time step to sync them
    pub fn end_step(particles: &mut Vec<Particle>, next_sync_step: f64) {
        for p in particles {
            Self::fast_forward(p, next_sync_step);
        }
    }

    fn compute_acc(
        &mut self,
        (p1i, p1): (ParticleIndex, &mut Particle),
        (p2i, p2): (ParticleIndex, &mut Particle),
        step_num: usize,
    ) -> (Vector, Vector, EventData) {
        // FIXME: this
        let current_impact_vel = Particle::impact_speed(p1, p2);
        let x_len = (Vector(p1.p) - Vector(p2.p)).mag();
        let x_hat = (Vector(p1.p) - Vector(p2.p)) / x_len;
        let separation_distance = x_len - p1.r - p2.r;
        let vji = Vector(p1.p) - Vector(p2.p);

        let impact_speed = if separation_distance < 0. {
            // colliding
            // look up impact velocity
            match self.impact_vel.borrow().get(p1i, p2i) {
                None => current_impact_vel,
                Some((speed, _)) => f64::max(speed, current_impact_vel),
            }
        } else {
            // not colliding
            current_impact_vel
        };

        debugln!("current_impact_vel={}", current_impact_vel);

        let reduced_mass = (p1.m * p2.m) / (p1.m + p2.m);

        use crate::rotter::b_and_k;
        let (b, k) = b_and_k(impact_speed, reduced_mass, f64::min(p1.r, p2.r));

        let info = EventData {
            impact_speed,
            b,
            k,
            separation_distance,
            reduced_mass,
        };

        if separation_distance < 0. {
            // colliding
            // spring-force
            let f_spring = x_hat * -k * separation_distance;
            let f_damp = vji * -b;

            let f_total = f_spring + f_damp;

            // if colliding, keep refreshing (or updating in case of a speedup) the impact vel tracker
            self.impact_vel
                .borrow_mut()
                .add(p1i, p2i, impact_speed, step_num);

            (f_total / p1.m, f_total / p2.m, info)
        } else {
            (
                Vector(calc_pp_accel(p1, p2)),
                Vector(calc_pp_accel(p2, p1)),
                info,
            )
        }
    }

    // the returned time will not be greater than next_sync_step
    // returns next_event_time, dt, whether to push to pq
    fn get_next_time(
        &self,
        separation_distance: f64,
        next_sync_step: f64,
        current_impact_vel: f64,
        event_time: f64,
        k: f64,
        m: f64,
    ) -> (f64, f64, PushPq) {
        use crate::no_explode::omega_0_from_k;

        let omega_0 = omega_0_from_k(k, m);
        assert!(omega_0 >= 0.);

        // TODO: abstract out gravity forces

        let mut dt = if separation_distance < 0. {
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
        };

        if dt < self.minimum_time_step {
            eprintln!(
                "WARN: time step too small: dt={}, event_time={}",
                dt, event_time
            );
        }

        let mut next_time = event_time + dt;

        // if moving at each other, reschedule at dt/2, but not less than big_time_step/100
        let repush_to_pq = if next_time > next_sync_step {
            next_time = next_sync_step;
            dt = next_time - event_time;
            PushPq::NoPush
        } else {
            PushPq::DoPush
        };

        (next_time, dt, repush_to_pq)
    }

    pub fn process_pair_get_dv(
        &mut self,
        (p1i, p1): (ParticleIndex, &mut Particle),
        (p2i, p2): (ParticleIndex, &mut Particle),
        step_num: usize,
        next_sync_step: f64,
        current_time: f64,
    ) -> (Vector, Vector) {
        let (acc1, acc2, info) = self.compute_acc((p1i, p1), (p2i, p2), step_num);

        let (next_time, dt, repush_to_pq) = self.get_next_time(
            info.separation_distance,
            next_sync_step,
            info.impact_speed,
            current_time,
            info.k,
            info.reduced_mass,
        );

        debugln!(
            "repush_to_pq={repush_to_pq:?} dt={dt} sep_dis={} impact_speed={}",
            info.separation_distance,
            info.impact_speed
        );

        let dvs = (
            acc1 * (next_time - current_time),
            acc2 * (next_time - current_time),
        );

        match repush_to_pq {
            PushPq::DoPush => {
                self.queue.push(ForceEvent {
                    time: next_time,
                    last_time: current_time,
                    p1: p1i,
                    p2: p2i,
                });
            }
            PushPq::NoPush => {
                // do nothing
            }
        }

        return dvs;
    }

    pub fn run_through_collision_pairs(
        &mut self,
        particles: &mut Vec<Particle>,
        next_sync_step: f64,
        step_num: usize,
    ) {
        loop {
            if let Some(event) = self.queue.pop() {
                // do stuff -> process collisions
                let (p1, p2) = borrow_two_elements(particles, event.p1.0, event.p2.0);

                Self::fast_forward(p1, event.time);
                Self::fast_forward(p2, event.time);

                let (dv1, dv2) = self.process_pair_get_dv(
                    (event.p1, p1),
                    (event.p2, p2),
                    step_num,
                    next_sync_step,
                    event.time,
                );

                p1.apply_dv(dv1);
                p2.apply_dv(dv2);
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
