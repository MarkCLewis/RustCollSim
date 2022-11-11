use std::{cell::RefCell, marker::PhantomData};

use crate::{
    kd_tree::{self, Interaction},
    particle::{Particle, ParticleIndex},
    soft_collision_queue::SoftSphereForce,
    util::borrow_two_elements,
    vectors::Vector,
};

pub struct KDTreeSystem {
    pop: RefCell<Vec<Particle>>,
    tree: kd_tree::KDTree,
    time_step: f64,
    current_time: f64,
    pub pq: RefCell<SoftSphereForce>,
    dv_tmp: RefCell<Vec<Vector>>,
}

impl KDTreeSystem {
    pub fn new(pop: Vec<Particle>, time_step: f64) -> Self {
        let size = pop.len();
        Self {
            tree: kd_tree::KDTree::new(pop.len()),
            pop: RefCell::new(pop),
            time_step,
            current_time: 0.,
            pq: RefCell::new(SoftSphereForce::new(time_step)),
            dv_tmp: {
                let mut v = Vec::new();
                v.resize(size, Vector::ZERO);
                RefCell::new(v)
            },
        }
    }

    pub fn apply_forces(&mut self, step_num: usize) {
        self.tree.rebuild_kdtree(&self.pop.borrow());

        let borrow_dv = &self.dv_tmp;

        let next_sync_step = self.current_time + self.time_step;

        let mut tree_map_func = |p: ParticleIndex, inter: Interaction| match inter {
            Interaction::ParticleParticle(other_idx) => {
                let mut pop_ref = self.pop.borrow_mut();
                let (p1, p2) = borrow_two_elements(&mut pop_ref, p.0, other_idx.0);

                let (dv1, _dv2) = self.pq.borrow_mut().process_pair_get_dv(
                    (p, p1),
                    (other_idx, p2),
                    step_num,
                    next_sync_step,
                    self.current_time,
                );

                let mut mut_dv = borrow_dv.borrow_mut();
                mut_dv[p.0] += dv1;
            }
            Interaction::ParticleNode(com, mass) => {
                let mut pop_ref = self.pop.borrow_mut();
                let particle = &mut pop_ref[p.0];
                let d_position = Vector(particle.p) - com;
                let d_sq = d_position * d_position;
                let d = d_sq.sqrt();
                let magi = -mass / (d * d_sq);

                let acc = d_position * magi;

                let dv = acc * self.time_step;

                let mut mut_dv = borrow_dv.borrow_mut();
                mut_dv[p.0] += dv;
            }
        };

        let len = self.pop.borrow().len();
        for p in 0..len {
            self.tree.map_tree(p, &self.pop, &mut tree_map_func);
        }
    }

    pub fn end_step(&mut self, step_count: usize) {
        let next_time = self.current_time + self.time_step;
        let mut pop_ref = self.pop.borrow_mut();
        self.pq
            .borrow_mut()
            .do_one_step(&mut pop_ref, next_time, step_count);
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
