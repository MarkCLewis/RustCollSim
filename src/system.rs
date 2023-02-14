use std::cell::RefCell;

use crate::{
    debugln,
    kd_tree::{self, Interaction},
    particle::{Particle, ParticleIndex},
    soft_collision_queue::SoftSphereForce,
    util::borrow_two_elements,
    vectors::Vector,
};

pub struct KDTreeSystem {
    pub pop: RefCell<Vec<Particle>>,
    tree: kd_tree::KDTree,
    time_step: f64,
    current_time: f64,
    pub pq: RefCell<SoftSphereForce>,
    dv_tmp: RefCell<Vec<Vector>>,
}

impl KDTreeSystem {
    pub fn new(pop: Vec<Particle>, time_step: f64, desired_collision_step_count: usize) -> Self {
        let size = pop.len();
        Self {
            tree: kd_tree::KDTree::new(pop.len()),
            pop: RefCell::new(pop),
            time_step,
            current_time: 0.,
            pq: RefCell::new(SoftSphereForce::new(
                time_step,
                desired_collision_step_count,
            )),
            dv_tmp: {
                let mut v = Vec::new();
                v.resize(size, Vector::ZERO);
                RefCell::new(v)
            },
        }
    }

    // pub fn with_tracing(&mut self, file_path: &str) {
    //     self.tracing_writer = Some(BufWriter::new(File::create(file_path).unwrap()));
    // }

    /// FIXME: this optional feature mess
    pub fn run(&mut self, steps: usize) {
        self.run_with_quit_option(
            steps,
            #[cfg(feature = "early_quit")]
            &mut |_| false,
        );
    }

    pub fn run_with_quit_option(
        &mut self,
        steps: usize,
        #[cfg(feature = "early_quit")] check_early_quit: &mut dyn FnMut(&[Particle]) -> bool,
    ) {
        for i in 0 as usize..steps {
            // println!("step: {}", i);
            self.apply_forces(i);
            self.end_step(
                i,
                #[cfg(feature = "early_quit")]
                check_early_quit,
            );

            #[allow(unused_variables)]
            for (p_idx, p) in self.pop.borrow().iter().enumerate() {
                debugln!(
                    "STEP {},{},{},{}",
                    self.current_time,
                    p_idx,
                    p.p.x(),
                    p.v.x()
                );
            }

            if i % 10 == 0 {
                self.pq.borrow_mut().trim_impact_vel_tracker(i);
            }

            #[cfg(feature = "early_quit")]
            if check_early_quit(&self.pop.borrow()) {
                break;
            }
        }
    }

    pub fn apply_forces(&mut self, step_num: usize) {
        self.tree.rebuild_kdtree(&self.pop.borrow());

        let borrow_dv = &self.dv_tmp;

        let next_sync_step = self.current_time + self.time_step;

        let mut tree_map_func = |p: ParticleIndex, inter: Interaction| match inter {
            Interaction::ParticleParticle(other_idx) => {
                debugln!("{p} {other_idx}",);
                let mut pop_ref = self.pop.borrow_mut();
                let (p1, p2) = borrow_two_elements(&mut pop_ref, p.0, other_idx.0);

                let (dv1, dv2) = self.pq.borrow_mut().process_pair_get_dv(
                    (p, p1),
                    (other_idx, p2),
                    step_num,
                    next_sync_step,
                    self.current_time,
                );

                let mut mut_dv = borrow_dv.borrow_mut();
                mut_dv[p.0] += dv1;
                mut_dv[other_idx.0] += dv2;
            }
            Interaction::ParticleNode(com, mass) => {
                if cfg!(feature = "no_gravity") {
                    return;
                }
                let mut pop_ref = self.pop.borrow_mut();
                let particle = &mut pop_ref[p.0];
                let d_position = particle.p - com;
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

        let dv_ref = &mut borrow_dv.borrow_mut();

        let pop = &mut self.pop.borrow_mut();
        for (p, dv) in pop.iter_mut().zip(dv_ref.iter()) {
            p.apply_dv(*dv);
        }

        for dv in dv_ref.iter_mut() {
            *dv = Vector::ZERO;
        }
    }

    pub fn end_step(
        &mut self,
        step_count: usize,
        #[cfg(feature = "early_quit")] check_early_quit: &mut dyn FnMut(&[Particle]) -> bool,
    ) {
        let next_time = self.current_time + self.time_step;
        let mut pop_ref = self.pop.borrow_mut();

        self.pq.borrow_mut().do_one_step(
            &mut pop_ref,
            next_time,
            step_count,
            #[cfg(feature = "early_quit")]
            check_early_quit,
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

// /// A force that can be applied to a population
// pub trait Force<Population> {
//     fn apply_force(self, p: &mut Population);
// }

// /// Combines two forces into one by applying one, then the other
// pub struct DoubleForce<P, F1: Force<P>, F2: Force<P>> {
//     pub f1: F1,
//     pub f2: F2,
//     ph: PhantomData<P>,
// }

// impl<P, F1: Force<P>, F2: Force<P>> DoubleForce<P, F1, F2> {
//     pub fn new(f1: F1, f2: F2) -> Self {
//         DoubleForce {
//             f1,
//             f2,
//             ph: PhantomData,
//         }
//     }
// }

// impl<P, F1: Force<P>, F2: Force<P>> Force<P> for DoubleForce<P, F1, F2> {
//     fn apply_force(self, p: &mut P) {
//         self.f1.apply_force(p);
//         self.f2.apply_force(p);
//     }
// }

// pub struct NullForce {}

// impl<P> Force<P> for NullForce {
//     fn apply_force(self, _p: &mut P) {}
// }
