use std::marker::PhantomData;

use crate::{
    kd_tree::{self, Interaction},
    particle::Particle,
    soft_collision_queue::ForceQueue,
    vectors::Vector,
};

pub struct KDTreeSystem<F>
where
    F: Fn(&mut Particle, &mut Particle) -> ([f64; 3], [f64; 3]),
{
    pop: Vec<Particle>,
    tree: kd_tree::KDTree,
    time_step: f64,
    current_time: f64,
    pq: ForceQueue<F>,
    dv_tmp: Vec<Vector>,
}

impl<F> KDTreeSystem<F>
where
    F: Fn(&mut Particle, &mut Particle) -> ([f64; 3], [f64; 3]),
{
    pub fn new(pop: Vec<Particle>, time_step: f64, pair_force: F) -> Self {
        let size = pop.len();
        Self {
            tree: kd_tree::KDTree::new(pop.len()),
            pop,
            time_step,
            current_time: 0.,
            pq: ForceQueue::new(time_step, pair_force),
            dv_tmp: {
                let mut v = Vec::new();
                v.resize(size, Vector::ZERO);
                v
            },
        }
    }

    pub fn apply_forces(&mut self) {
        self.tree.rebuild_kdtree(&self.pop);

        let mut dv_apply =
            |p: usize, particle: &Particle, inter: Interaction, p_acc: Vector| match inter {
                Interaction::ParticleParticle(other) => {}
                Interaction::ParticleNode => {
                    let dv = p_acc * self.time_step;
                    self.dv_tmp[p] += dv;
                }
            };

        for p in 0..self.pop.len() {
            self.tree.map_calc_acc(p, &self.pop, &mut dv_apply);
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
