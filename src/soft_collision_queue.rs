use std::{cell::RefCell, cmp::Ordering, collections::BinaryHeap};

use indicatif::ProgressBar;

use crate::{
    debugln,
    impact_vel_tracker::ImpactVelocityTracker,
    no_explode::SpringDerivation,
    particle::{calc_pp_accel, Particle, ParticleIndex},
    util::borrow_two_elements,
    vectors::Vector,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExitReason {
    NormalEnd,
    #[allow(dead_code)]
    TerminatedEarly,
}

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
        self.time.partial_cmp(&other.time).map(Ordering::reverse) // flipping order as binary heap is max heap
    }
}
impl Eq for ForceEvent {} // time better not be NaN or infinity, otherwise this breaks
impl Ord for ForceEvent {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).expect(&format!(
            "Got invalid timestamp in ForceEvent: {} and {}",
            self.time, other.time
        ))
    }
}

pub struct SoftSphereForce<S: SpringDerivation> {
    pub queue: BinaryHeap<ForceEvent>,
    desired_collision_step_count: usize,
    warn_minimum_time_step: Option<f64>, // print warn if dt < this
    minimum_time_step: Option<f64>,      // a floor to how small dt can be
    pub impact_vel: RefCell<ImpactVelocityTracker>,
    spring_derivation: S,
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

impl<S: SpringDerivation> SoftSphereForce<S> {
    pub fn new(
        big_time_step: f64,
        desired_collision_step_count: usize,
        spring_derivation: S,
    ) -> Self {
        Self {
            queue: BinaryHeap::new(),
            desired_collision_step_count,
            minimum_time_step: Some(big_time_step / 100.),
            warn_minimum_time_step: Some(big_time_step / 80.),
            impact_vel: RefCell::new(ImpactVelocityTracker::new()),
            spring_derivation,
        }
    }

    pub fn no_small_dt_warn(&mut self) -> &mut Self {
        self.warn_minimum_time_step = None;
        self
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

        particle.p += particle.v * dt;
        particle.t = current_time;
    }

    pub fn do_one_step(
        &mut self,
        particles: &mut Vec<Particle>,
        next_sync_step: f64,
        last_sync_step: f64,
        relative_speed_estimate: f64,
        disable_pq: bool,
        progress_bar: Option<&ProgressBar>,
        step_count: usize,
        #[cfg(feature = "early_quit")] check_early_quit: &mut dyn FnMut(&[Particle]) -> bool,
    ) -> ExitReason {
        let exit_reason = self.run_through_collision_pairs(
            particles,
            next_sync_step,
            last_sync_step,
            relative_speed_estimate,
            disable_pq,
            progress_bar,
            step_count,
            #[cfg(feature = "early_quit")]
            check_early_quit,
        );

        if let ExitReason::TerminatedEarly = exit_reason {
            return exit_reason;
        }

        Self::end_step(particles, next_sync_step);
        ExitReason::NormalEnd
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
        let current_relative_speed = Particle::relative_speed(p1, p2);
        let x_len = (p1.p - p2.p).mag();
        let x_hat = (p1.p - p2.p) / x_len;
        let separation_distance = x_len - p1.r - p2.r;
        let vji = p1.v - p2.v;

        // if p1i.0 == 34 && p2i.0 == 210 && step_num >= 60 {
        //     // && p1.p.y() > 0.00001153 && p1.p.x() < -5.79e-7
        //     // && p1.p.x() > 0.000009 {
        //     // unsafe { breakpoint() }
        //     // eprintln!("P{} {} {}", p1i.0, p1.p.x(), p1.p.y());
        //     // eprintln!("P{} {} {}", p2i.0, p2.p.x(), p2.p.y());

        //     if step_num > 75 {
        //         unsafe { breakpoint() }
        //     }
        // }

        // if (p1i.0 == 34 || p2i.0 == 34 || p1i.0 == 210 || p2i.0 == 210)
        //     && step_num >= 75
        //     && separation_distance < 0.
        // {
        //     eprintln!("{} {}", p1i.0, p2i.0);
        // }

        // if separation_distance < 0.
        //     && (p1.p.x() == -0.0000005769515679736722 || p2.p.x() == -0.0000005769515679736722)
        // {
        //     unsafe { breakpoint() }
        // }

        // if separation_distance < 0. && (separation_distance / (p1.r + p2.r)).abs() > 0.03 {
        //     println!(
        //         "WARN: separation_distance = {:.2}%\tIndices: p1 = {}, p2 = {}, step_num = {}",
        //         (separation_distance / (p1.r + p2.r)).abs() * 100.,
        //         p1i.0,
        //         p2i.0,
        //         step_num
        //     );

        //     if (separation_distance / (p1.r + p2.r)).abs() > 0.1 {
        //         panic!();
        //     }
        // }

        // for (idx, particle) in [(p1i.0, &p1), (p2i.0, &p2)] {
        //     if idx == 42 || idx == 283 || idx == 94 {
        //         eprintln!("P{} {} {}", idx, particle.p.x(), particle.p.y())
        //     }
        // }

        let impact_speed = if separation_distance < 0. {
            // colliding
            // look up impact velocity
            match self.impact_vel.borrow().get(p1i, p2i) {
                None => current_relative_speed,
                Some((last_known_impact_speed, _)) => {
                    f64::max(last_known_impact_speed, current_relative_speed)
                }
            }
        } else {
            // not colliding
            current_relative_speed
        };

        debugln!("current_impact_vel={}", impact_speed);

        let reduced_mass = (p1.m * p2.m) / (p1.m + p2.m);

        let (b, k) =
            self.spring_derivation
                .b_and_k(impact_speed, reduced_mass, f64::min(p1.r, p2.r));

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

            debugln!("{} {}", f_spring.mag(), f_damp.mag());

            let f_total = f_spring + f_damp;

            // if colliding, keep refreshing (or updating in case of a speedup) the impact vel tracker
            self.impact_vel.borrow_mut().add(
                p1i,
                p2i,
                impact_speed,
                step_num,
                #[cfg(test)]
                true,
                #[cfg(test)]
                (-separation_distance).max(0.), // penetration depth, as a positive number. If not colliding, this is 0.
                #[cfg(test)]
                current_relative_speed,
            );

            (f_total / p1.m, -f_total / p2.m, info)
        } else {
            self.impact_vel.borrow_mut().remove(p1i, p2i);
            // idea:? when getting the impact vel, make sure its not too old, like only 1 step old max

            if cfg!(feature = "no_gravity") {
                (Vector::ZERO, Vector::ZERO, info)
            } else {
                // gravity
                (calc_pp_accel(p1, p2), calc_pp_accel(p2, p1), info)
            }
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
        b: f64,
        relative_speed_estimate: f64,
        r1: f64,
        r2: f64,
        disable_pq: bool,
    ) -> (f64, f64, PushPq) {
        if disable_pq {
            return (next_sync_step, next_sync_step - event_time, PushPq::NoPush);
        }

        use crate::no_explode::omega_l;

        // let omega_0 = omega_0_from_k(k, m);
        // assert!(omega_0 >= 0.);

        let omega_l = omega_l(k, m, b);

        // T = 1/f = 2\pi/\omega
        // Time of collision is T/2
        let collision_time = std::f64::consts::PI / omega_l;
        let collision_time_dt = collision_time / self.desired_collision_step_count as f64;

        // NOTE: this right here injects relative_speed_estimate into the collision time calculation
        let current_impact_speed = current_impact_vel.abs();

        // TODO: abstract out gravity forces

        // how far two particles can intersect without things getting out of hand
        let max_ok_pen_estimate = f64::max(r1, r2) * self.spring_derivation.get_pen_fraction();

        let (mut dt, distance_for_global_speed_estimate) = if separation_distance < 0. {
            debugln!("colliding",);
            // colliding

            (collision_time_dt, max_ok_pen_estimate)
        } else {
            // v * t = d
            let impact_time_dt = separation_distance / current_impact_speed;

            // max( dist/(2*v_normal) and 1/(\omega_0 C) )

            // should this be min?
            // No: otherwise we get a zeno's paradox
            // the collision should be processed at steps of dt
            (
                f64::max(
                    impact_time_dt.abs() / 2.,
                    collision_time_dt, // 1. / (omega_l * self.desired_collision_step_count as f64),
                ),
                f64::max(separation_distance, max_ok_pen_estimate),
            )
        };

        // impact_time_estimate is to ensure that if there is some fast moving particles around,
        // dt will be small enough such that if one particle in this pair gets hit, this pair will get updated properly
        // this is if one of the fastest particles suddenly crashes into one of the pair of particles
        // processed here and transfers all its speed
        let impact_time_dt_from_speed_estimate =
            distance_for_global_speed_estimate / relative_speed_estimate;

        dt = f64::min(dt, impact_time_dt_from_speed_estimate);

        if let Some(warn_minimum_time_step) = self.warn_minimum_time_step {
            if dt < warn_minimum_time_step {
                eprintln!(
                    "WARN: time step too small: dt={}, event_time={}",
                    dt, event_time
                );
            }
        }

        if let Some(minimum_time_step) = self.minimum_time_step {
            dt = f64::max(dt, minimum_time_step);
        }

        if dt == 0. {
            panic!(
                "dt is 0. This should not happen and will create an infinite loop.\n separation distance: {}\n next_sync_step: {}\n current_impact_vel: {}\n event_time: {}\n k: {}\n m: {}\n b: {}\n",
                separation_distance, next_sync_step, current_impact_vel, event_time, k, m, b
            );
        }

        if event_time + dt == event_time {
            panic!(
                "dt is so small its a rounding error.\nevent_time + dt == event_time. This should not happen and will create an infinite loop.\n separation distance: {}\n next_sync_step: {}\n current_impact_vel: {}\n event_time: {}\n k: {}\n m: {}\n b: {}\n",
                separation_distance, next_sync_step, current_impact_vel, event_time, k, m, b)
        }

        if let Some(minimum_time_step) = self.minimum_time_step {
            assert!(dt >= minimum_time_step);
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
        relative_speed_estimate: f64,
        disable_pq: bool,
    ) -> (Vector, Vector) {
        let (acc1, acc2, info) = self.compute_acc((p1i, p1), (p2i, p2), step_num);

        #[allow(unused_variables)]
        let (next_time, dt, repush_to_pq) = self.get_next_time(
            info.separation_distance,
            next_sync_step,
            info.impact_speed,
            current_time,
            info.k,
            info.reduced_mass,
            info.b,
            relative_speed_estimate,
            p1.r,
            p2.r,
            disable_pq,
        );

        debugln!(
            "repush_to_pq={repush_to_pq:?} dt={dt} sep_dis={} impact_speed={}",
            info.separation_distance,
            info.impact_speed
        );

        debugln!(
            "SUB_STEP {},{},{},{},{},{},{}",
            current_time,
            p1i.0,
            p1.p.x(),
            p1.v.x(),
            p2i.0,
            p2.p.x(),
            p2.v.x()
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

    fn run_through_collision_pairs(
        &mut self,
        particles: &mut Vec<Particle>,
        next_sync_step: f64,
        last_sync_step: f64,
        relative_speed_estimate: f64,
        disable_pq: bool,
        progress_bar: Option<&ProgressBar>,
        step_num: usize,
        #[cfg(feature = "early_quit")] check_early_quit: &mut dyn FnMut(&[Particle]) -> bool,
    ) -> ExitReason {
        if let Some(pb) = progress_bar {
            pb.set_position(0);
            pb.set_length(100);
        }

        let mut events_processed: u64 = 0;

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
                    relative_speed_estimate,
                    disable_pq,
                );

                p1.apply_dv(dv1);
                p2.apply_dv(dv2);

                // update progress bar
                events_processed += 1;
                if let Some(pb) = progress_bar {
                    let progress =
                        (event.time - last_sync_step) / (next_sync_step - last_sync_step);
                    pb.set_position((progress * 100.) as u64);
                    pb.set_message(format!("{}", events_processed));
                }

                #[cfg(feature = "early_quit")]
                if check_early_quit(particles) {
                    return ExitReason::TerminatedEarly;
                }
            } else {
                return ExitReason::NormalEnd;
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
