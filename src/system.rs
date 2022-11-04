use std::marker::PhantomData;

use crate::{
    kd_tree::{allocate_node_vec, build_tree, calc_accel, KDTree},
    particle::Particle,
    soft_collision_queue::ForceQueue,
    vectors::Vector,
};

pub struct KDTreeSystem<F>
where
    F: Fn(&mut Particle, &mut Particle) -> ([f64; 3], [f64; 3]),
{
    pop: Vec<Particle>,
    indices: Vec<usize>,
    tree: Vec<KDTree>,
    time_step: f64,
    current_time: f64,
    pq: ForceQueue<F>,
}

impl<F> KDTreeSystem<F>
where
    F: Fn(&mut Particle, &mut Particle) -> ([f64; 3], [f64; 3]),
{
    pub fn new(pop: Vec<Particle>, time_step: f64, pair_force: F) -> Self {
        Self {
            indices: (0..pop.len()).collect(),
            tree: allocate_node_vec(pop.len()),
            pop,
            time_step,
            current_time: 0.,
            pq: ForceQueue::new(time_step, pair_force),
        }
    }

    fn rebuild_kdtree(&mut self) {
        build_tree(
            &mut self.indices,
            0,
            self.pop.len(),
            &self.pop,
            0,
            &mut self.tree,
        );
    }

    pub fn apply_forces(&mut self) {
        self.rebuild_kdtree();

        for i in 0..self.pop.len() {
            apply_and_calc_forces(i, &self.pop, &self.tree);
        }
    }

    pub fn end_step(&mut self) {
        let next_time = self.current_time + self.time_step;
        self.pq.do_one_step(&mut self.pop, next_time);
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
