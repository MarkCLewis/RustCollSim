use std::{cell::RefCell, fs::File, io::Write};

use crate::{
    // debug_tools::{ChangeInspector, VelocityInspector},
    debugln,
    hills_force::{HillsForce, SlidingBrickBoundary},
    kd_tree::{self, Interaction},
    forces::no_explode,
    particle::{Particle, ParticleIndex},
    soft_collision_queue::{ExitReason, SoftSphereForce},
    util::{borrow_two_elements, progress_tracker::TwoPartProgress},
    vectors::Vector,
};

pub struct KDTreeSystem {
    pub pop: RefCell<Vec<Particle>>,
    tree: kd_tree::KDTree,
    time_step: f64,
    current_time: f64,
    pub pq: RefCell<SoftSphereForce<no_explode::Rotter>>,
    dv_tmp: RefCell<Vec<Vector>>,
    hills_force: Option<HillsForce>,
    sliding_brick: Option<SlidingBrickBoundary>,
    serialize_run: Option<File>,
    progress_bar: Option<TwoPartProgress>,
    disable_pq: bool,
}

impl KDTreeSystem {
    pub fn new(
        pop: Vec<Particle>,
        time_step: f64,
        desired_collision_step_count: usize,
        desired_coefficient_of_restitution: f64,
        opts: Option<&crate::Opts>,
    ) -> Self {
        let default_pen_fraction = 0.02;
        let size = pop.len();

        //no_small_dt_warn

        let mut pq = SoftSphereForce::new(
            time_step,
            desired_collision_step_count,
            no_explode::Rotter::new(desired_coefficient_of_restitution, default_pen_fraction),
        );

        if let Some(opts) = opts {
            if opts.no_warnings {
                pq.no_small_dt_warn();
            }
        }

        Self {
            tree: kd_tree::KDTree::new(pop.len()),
            pop: RefCell::new(pop),
            time_step,
            current_time: 0.,
            pq: RefCell::new(pq),
            dv_tmp: {
                let mut v = Vec::new();
                v.resize(size, Vector::ZERO);
                RefCell::new(v)
            },
            hills_force: None,
            sliding_brick: None,
            serialize_run: None,
            progress_bar: None,
            disable_pq: false,
        }
    }

    pub fn set_hills_force(mut self: Self, hills_force: HillsForce) -> Self {
        self.hills_force = Some(hills_force);
        self
    }

    pub fn set_sliding_brick(mut self: Self, sliding_brick: SlidingBrickBoundary) -> Self {
        self.sliding_brick = Some(sliding_brick);
        self
    }

    pub fn set_serialize_run(mut self: Self, serialize_run_file: Option<File>) -> Self {
        self.serialize_run = serialize_run_file;
        self
    }

    pub fn set_progress_bar(mut self: Self, progress_bar: Option<TwoPartProgress>) -> Self {
        self.progress_bar = progress_bar;
        self
    }

    pub fn set_disable_pq(mut self: Self, disable_pq: bool) -> Self {
        eprintln!("disable_pq: {}", disable_pq);
        self.disable_pq = disable_pq;
        self
    }

    /// FIXME: this optional feature mess
    pub fn run(&mut self, steps: usize) -> ExitReason {
        self.run_with_quit_option(
            steps,
            #[cfg(feature = "early_quit")]
            &mut |_| false,
        )
    }

    /// main simulation loop
    pub fn run_with_quit_option(
        &mut self,
        steps: usize,
        #[cfg(feature = "early_quit")] check_early_quit: &mut dyn FnMut(&[Particle]) -> bool,
    ) -> ExitReason {
        self.attempt_serialize(0);

        eprintln!("Running for {} steps", steps);
        eprintln!("Running for time {}", steps as f64 * self.time_step);

        for i in 0 as usize..steps {
            if let Some(pb) = &self.progress_bar {
                pb.incr_main(1);
                pb.sub_bar().map(|pb| {
                    pb.set_length(6);
                    pb.set_position(0);
                });
                pb.set_sub_message("Hills Force...");
            }
            // let mut change_inspector = VelocityInspector::before(&self.pop, 10.);

            if let Some(hills_force) = &self.hills_force {
                hills_force.apply_delta_velocity(&mut self.pop.borrow_mut(), self.time_step);
            }

            if let Some(pb) = &self.progress_bar {
                pb.incr_sub(1);
                pb.set_sub_message("kD-tree walk...");
            }

            // change_inspector.after(&self.pop);

            let relative_speed_estimate = self.apply_forces(i);

            if let Some(pb) = &self.progress_bar {
                pb.incr_sub(1);
                pb.set_sub_message("Priority Queue...");
            }

            let exit_reason = self.end_step(
                i,
                relative_speed_estimate,
                #[cfg(feature = "early_quit")]
                check_early_quit,
            );

            if let Some(pb) = &self.progress_bar {
                pb.incr_sub(1);
                pb.set_sub_message("Sliding Brick...");
            }

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

            if let ExitReason::TerminatedEarly = exit_reason {
                return exit_reason;
            }

            #[cfg(feature = "early_quit")]
            if check_early_quit(&self.pop.borrow()) {
                return ExitReason::TerminatedEarly;
            }

            if let Some(sliding_brick) = &self.sliding_brick {
                sliding_brick.apply(&mut self.pop.borrow_mut(), self.current_time);
            }

            if let Some(pb) = &self.progress_bar {
                pb.incr_sub(1);
                pb.set_sub_message("Trimming impact v...");
            }

            // if i % 10 == 0 {
            //     self.pq.borrow_mut().trim_impact_vel_tracker(i);
            // }

            if let Some(pb) = &self.progress_bar {
                pb.incr_sub(1);
                pb.set_sub_message("serializing...");
            }

            self.attempt_serialize(i + 1);

            if let Some(pb) = &self.progress_bar {
                pb.incr_sub(1);
                pb.set_sub_message("done");
            }
        }

        if let Some(pb) = &self.progress_bar {
            pb.main_bar().finish();
        }

        ExitReason::NormalEnd
    }

    pub fn apply_forces(&mut self, step_num: usize) -> f64 {
        self.tree.rebuild_kdtree(&self.pop.borrow());

        let relative_speed_estimate = self
            .tree
            .global_relative_speed_estimate_rms(&self.pop.borrow())
            * 2.0;

        // global_relative_speed_estimate_rms

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
                    relative_speed_estimate,
                    self.disable_pq,
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

        return relative_speed_estimate;
    }

    pub fn end_step(
        &mut self,
        step_count: usize,
        relative_speed_estimate: f64,
        #[cfg(feature = "early_quit")] check_early_quit: &mut dyn FnMut(&[Particle]) -> bool,
    ) -> ExitReason {
        let next_time = self.current_time + self.time_step;
        let mut pop_ref = self.pop.borrow_mut();

        let exit_reason = self.pq.borrow_mut().do_one_step(
            &mut pop_ref,
            next_time,
            self.current_time,
            relative_speed_estimate,
            self.disable_pq,
            match &self.progress_bar {
                Some(pb) => pb.sub_sub_bar(),
                None => None,
            },
            step_count,
            #[cfg(feature = "early_quit")]
            check_early_quit,
        );

        if let ExitReason::TerminatedEarly = exit_reason {
            return exit_reason;
        }

        self.current_time = next_time;

        ExitReason::NormalEnd
    }

    fn serialize(&self, step: usize) -> String {
        let mut s = String::new();
        for p in self.pop.borrow().iter() {
            s.push_str(&format!(
                "{} {} {} {} {}\n",
                p.p.x(),
                p.p.y(),
                p.p.z(),
                p.r,
                step
            ));
        }
        s
    }

    fn attempt_serialize(&mut self, step: usize) {
        if self.serialize_run.is_none() {
            return;
        }

        let s = self.serialize(step);
        if let Some(f) = &mut self.serialize_run {
            f.write_all(s.as_bytes()).unwrap();
        }
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
