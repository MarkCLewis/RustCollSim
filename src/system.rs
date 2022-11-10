use std::{cell::RefCell, marker::PhantomData};

use crate::{
    impact_vel_tracker::ImpactVelocityTracker,
    kd_tree::{self, Interaction},
    particle::{Particle, ParticleIndex},
    soft_collision_queue::{ForceEvent, ForceQueue, PushPq},
    vectors::Vector,
};

pub struct KDTreeSystem<F>
where
    F: Fn(
        &mut Particle,
        &mut Particle,
        &mut ImpactVelocityTracker,
        ParticleIndex,
        ParticleIndex,
        usize,
    ) -> (Vector, Vector),
{
    pop: Vec<Particle>,
    tree: kd_tree::KDTree,
    time_step: f64,
    current_time: f64,
    pq: RefCell<ForceQueue<F>>,
    dv_tmp: RefCell<Vec<Vector>>,
    impact_vel: RefCell<ImpactVelocityTracker>,
}

impl<F> KDTreeSystem<F>
where
    F: Fn(
        &mut Particle,
        &mut Particle,
        &mut ImpactVelocityTracker,
        ParticleIndex,
        ParticleIndex,
        usize,
    ) -> (Vector, Vector),
{
    pub fn new(pop: Vec<Particle>, time_step: f64, pair_force: F) -> Self {
        let size = pop.len();
        Self {
            tree: kd_tree::KDTree::new(pop.len()),
            pop,
            time_step,
            current_time: 0.,
            pq: RefCell::new(ForceQueue::new(time_step, pair_force)),
            dv_tmp: {
                let mut v = Vec::new();
                v.resize(size, Vector::ZERO);
                RefCell::new(v)
            },
            impact_vel: RefCell::new(ImpactVelocityTracker::new()),
        }
    }

    pub fn trim_impact_vel_tracker(&mut self, current_step: usize) {
        self.impact_vel.borrow_mut().trim(current_step);
    }

    pub fn apply_forces(&mut self, step_count: usize) {
        self.tree.rebuild_kdtree(&self.pop);

        let borrow_dv = &self.dv_tmp;

        let next_sync_step = self.current_time + self.time_step;

        let mut dv_apply = |p: ParticleIndex,
                            particle: &Particle,
                            inter: Interaction,
                            p_acc: Vector| match inter {
            Interaction::ParticleParticle(other_idx, other) => {
                let current_impact_vel = Particle::impact_speed(particle, other);

                let updated_impact_vel = match self.impact_vel.borrow().get(p, other_idx) {
                    Some((v, _)) => f64::max(v, current_impact_vel),
                    None => current_impact_vel,
                };

                let mut mut_borrow = self.pq.borrow_mut();

                let (next_event_time, dt, push_pq) = mut_borrow.get_next_time(
                    (Vector(particle.p) - Vector(other.p)).mag(),
                    next_sync_step,
                    updated_impact_vel,
                    self.current_time,
                    particle.m,
                    particle.r,
                );

                let dv = p_acc * dt;
                {
                    let mut mut_dv = borrow_dv.borrow_mut();
                    mut_dv[p.0] += dv;
                }
                match push_pq {
                    PushPq::DoPush => {
                        mut_borrow.queue.push(ForceEvent {
                            time: next_event_time,
                            last_time: self.current_time,
                            p1: p,
                            p2: other_idx,
                        });
                    }
                    PushPq::NoPush => {
                        // do nothing
                    }
                }
                self.impact_vel
                    .borrow_mut()
                    .add(p, other_idx, updated_impact_vel, step_count);
            }
            Interaction::ParticleNode => {
                let dv = p_acc * self.time_step;

                let mut mut_dv = borrow_dv.borrow_mut();
                mut_dv[p.0] += dv;
            }
        };

        for p in 0..self.pop.len() {
            self.tree.map_calc_acc(p, &self.pop, &mut dv_apply);
        }
    }

    pub fn end_step(&mut self, step_count: usize) {
        let next_time = self.current_time + self.time_step;
        self.pq.borrow_mut().do_one_step(
            &mut self.pop,
            next_time,
            &mut self.impact_vel.borrow_mut(),
            step_count,
        );
        self.current_time = next_time;
    }
}

// pub struct System<'a, Population, F: Force<Population>> {
//     pop: &'a mut Population,
//     f: F,
// }

// impl<'a, Population, F: Force<Population>> System<'a, Population, F> {
//     pub fn new(pop: &'a mut Population, f: F) -> Self {
//         Self { pop, f }
//     }
// }

/// A force that can be applied to a population
pub trait Force<Population> {
    fn apply_force(self, p: &mut Population);
}

/// Combines two forces into one by applying one, then the other
pub struct DoubleForce<P, F1: Force<P>, F2: Force<P>> {
    pub f1: F1,
    pub f2: F2,
    ph: PhantomData<P>,
}

impl<P, F1: Force<P>, F2: Force<P>> DoubleForce<P, F1, F2> {
    pub fn new(f1: F1, f2: F2) -> Self {
        DoubleForce {
            f1,
            f2,
            ph: PhantomData,
        }
    }
}

impl<P, F1: Force<P>, F2: Force<P>> Force<P> for DoubleForce<P, F1, F2> {
    fn apply_force(self, p: &mut P) {
        self.f1.apply_force(p);
        self.f2.apply_force(p);
    }
}

pub struct NullForce {}

impl<P> Force<P> for NullForce {
    fn apply_force(self, _p: &mut P) {}
}
