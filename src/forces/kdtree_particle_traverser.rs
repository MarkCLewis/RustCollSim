use std::ops::AddAssign;

use rayon::iter::{IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator};
use rayon::{join, prelude::*};

use crate::forces::quickstat::quickstat_index;
use crate::vectors::Axis;
use crate::{
    design::system::{BoundaryCondition, Particle, Population},
    forces::single_particle_event_force::{EventForce, Traverser},
    vectors::Vector,
};

const MAX_PARTS: usize = 8;
const THETA: f64 = 0.3;
const NEGS: [usize; MAX_PARTS] = [usize::MAX; MAX_PARTS];

#[derive(Clone, Copy, Debug)]
pub enum KDTree {
    Leaf {
        num_parts: usize,
        leaf_parts: [usize; MAX_PARTS],
    },

    Internal {
        split_dim: Axis,
        split_val: f64,
        m: f64,
        cmx: Vector,
        cmv: Vector,
        size: f64,
        left: usize,
        right: usize,
    },
}

impl KDTree {
    pub fn leaf(num_parts: usize, particles: [usize; MAX_PARTS]) -> KDTree {
        KDTree::Leaf {
            num_parts: num_parts,
            leaf_parts: particles,
        }
    }

    pub fn cm_data(&self, particles: &[Particle]) -> (f64, Vector, Vector) {
        match *self {
            KDTree::Leaf {
                num_parts,
                leaf_parts,
            } => {
                let mut m = 0.0;
                let mut cmx = Vector::new(0.0, 0.0, 0.0);
                let mut cmv = Vector::new(0.0, 0.0, 0.0);
                for i in 0..num_parts {
                    let p = &particles[leaf_parts[i]];
                    m += p.m;
                    cmx += p.x * p.m;
                    cmv += p.v * p.m;
                }
                (m, cmx / m, cmv / m)
            }
            KDTree::Internal {
                split_dim: _,
                split_val: _,
                m,
                cmx,
                cmv,
                size: _,
                left: _,
                right: _,
            } => (m, cmx, cmv),
        }
    }
}

pub struct KDTreeParticleTraverser {
    nodes: Vec<KDTree>,
}

fn nodes_needed_for_particles(num_parts: usize) -> usize {
    if num_parts <= MAX_PARTS {
        1
    } else {
        let min_num_leaves = num_parts / (MAX_PARTS / 2);
        let num_leaves = usize::pow(2, f64::log2(min_num_leaves as f64).ceil() as u32);
        2 * num_leaves - 1
    }
}

pub fn allocate_node_vec(num_parts: usize) -> Vec<KDTree> {
    let num_nodes = nodes_needed_for_particles(num_parts);
    let mut ret = Vec::new();
    ret.resize(num_nodes, KDTree::leaf(0, NEGS));
    ret
}

pub fn build_tree_par3_chunk(
    indices: &mut [usize],
    cur_node: usize,
    particles: &[Particle],
    nodes: &mut [KDTree],
    thread_cnt: usize,
) {
    // println!("indices: {:?} cur: {} nodes: {}", indices, cur_node, nodes.len());
    let np = indices.len();
    if np <= MAX_PARTS {
        let mut parts = [0; MAX_PARTS];
        for i in 0..np {
            parts[i] = indices[i]
        }
        nodes[0] = KDTree::Leaf {
            num_parts: np,
            leaf_parts: parts,
        };
        // println!("nodes[0] = {:?}", nodes[0]);
    } else {
        // Pick split dim and value
        let (min, max) = indices
            .par_chunks(500000)
            .fold(
                || {
                    (
                        // 0.0,
                        // Vector::new(0.0, 0.0, 0.0),
                        Vector::new(1e100, 1e100, 1e100),
                        Vector::new(-1e100, -1e100, -1e100),
                    )
                },
                |(min, max), chunk| {
                    let mut min = Vector::new(1e100, 1e100, 1e100);
                    let mut max = Vector::new(-1e100, -1e100, -1e100);
                    // let mut m = 0.0;
                    // let mut cmx = Vector::new(0.0, 0.0, 0.0);
                    // let mut cmv = Vector::new(0.0, 0.0, 0.0);
                    for i in chunk {
                        // m += particles[*i].m;
                        // cmx += particles[*i].x * particles[*i].m;
                        // cmv += particles[*i].v * particles[*i].m;
                        min = min.min_of_every_axis(&particles[*i].x);
                        max = max.max_of_every_axis(&particles[*i].x);
                    }
                    (min, max)
                },
            )
            .reduce(
                || {
                    (
                        // 0.0,
                        // Vector::new(0.0, 0.0, 0.0),
                        Vector::new(1e100, 1e100, 1e100),
                        Vector::new(-1e100, -1e100, -1e100),
                    )
                },
                |(min1, max1), (min2, max2)| {
                    (
                        // m1 + m2,
                        // cm1 + cm2,
                        min1.min_of_every_axis(&min2),
                        max1.max_of_every_axis(&max2),
                    )
                },
            );
        let mut split_dim = Axis::X;
        if max[Axis::Y] - min[Axis::Y] > max[split_dim] - min[split_dim] {
            split_dim = Axis::Y
        }
        if max[Axis::Z] - min[Axis::Z] > max[split_dim] - min[split_dim] {
            split_dim = Axis::Z
        }
        let size = max[split_dim] - min[split_dim];
        // println!("cur_node = {}, size = {}, split_dim={:?}, min={}, max={}",cur_node, size, split_dim, max[split_dim], min[split_dim]);

        // Partition particles on split_dim
        let mid = indices.len() / 2;
        quickstat_index(indices, mid, |i1, i2| {
            particles[i1].x[split_dim] < particles[i2].x[split_dim]
        });
        let split_val = particles[indices[mid]].x[split_dim];

        // Recurse on children and build this node.
        let (left_indices, right_indices) = indices.split_at_mut(mid);
        let num_nodes = nodes_needed_for_particles(left_indices.len());
        let (_this_node, other_nodes) = nodes.split_at_mut(1);
        let (left_nodes, right_nodes) = other_nodes.split_at_mut(num_nodes);
        // println!("num_nodes = {}", num_nodes);
        let (_, _) = if thread_cnt < num_cpus::get() {
            join(
                || {
                    build_tree_par3_chunk(
                        left_indices,
                        cur_node + 1,
                        particles,
                        left_nodes,
                        thread_cnt * 2,
                    )
                },
                || {
                    build_tree_par3_chunk(
                        right_indices,
                        cur_node + 1 + num_nodes,
                        particles,
                        right_nodes,
                        thread_cnt * 2,
                    )
                },
            )
        } else {
            (
                build_tree_par3_chunk(
                    left_indices,
                    cur_node + 1,
                    particles,
                    left_nodes,
                    thread_cnt * 2,
                ),
                build_tree_par3_chunk(
                    right_indices,
                    cur_node + 1 + num_nodes,
                    particles,
                    right_nodes,
                    thread_cnt * 2,
                ),
            )
        };

        // TODO: Calculate the CM data from the children
        let (left_m, left_cmx, left_cmv) = left_nodes[0].cm_data(particles);
        let (right_m, right_cmx, right_cmv) = right_nodes[0].cm_data(particles);
        let m = left_m + right_m;
        let cmx = (left_cmx * left_m + right_cmx * right_m) / m;
        let cmv = (left_cmv * left_m + right_cmv * right_m) / m;
        nodes[0] = KDTree::Internal {
            split_dim,
            split_val,
            m,
            cmx,
            cmv,
            size,
            left: cur_node + 1,
            right: cur_node + 1 + num_nodes,
        };
    }
}

impl KDTreeParticleTraverser {
    pub fn new(num_parts: usize) -> KDTreeParticleTraverser {
        let mut nodes = allocate_node_vec(num_parts);
        KDTreeParticleTraverser { nodes }
    }

    fn accel_recur<F: EventForce>(
        &self,
        cur_node: usize,
        i1: usize,
        p1: &Particle,
        spd1: &mut F::SingleParticleData,
        force: &F,
        particles: &[Particle],
        ppds: &mut Vec<(usize, F::ParticlePairData)>,
        mirror_num: usize,
        offset_x: &Vector,
        offset_v: &Vector,
    ) -> Vector {
        let do_print = false && (i1 == 704 || i1==5);
        if do_print { println!("accel {} {}", i1, cur_node); }
        match self.nodes[cur_node] {
            KDTree::Leaf {
                num_parts,
                leaf_parts,
            } => {
                if do_print { println!("leaf {} {:?}", num_parts, leaf_parts); }
                let mut acc = Vector::new(0.0, 0.0, 0.0);
                for i in 0..(num_parts) {
                    let i2 = leaf_parts[i];
                    if i2 != i1 {
                        let mut p2 = particles[i2].clone();
                        p2.v += *offset_v;
                        p2.x += *offset_x + p2.v * (p1.time - p2.time);
                        let (d_acc, ppd) =
                            force.particle_particle_accel(i1, p1, i2, &p2, spd1, mirror_num);
                        acc += d_acc;
                        ppd.iter().for_each(|ppd| ppds.push((i2, *ppd)));
                    }
                }
                if do_print { println!("leaf acc {}", acc); }
                acc
            }
            KDTree::Internal {
                m,
                cmx,
                cmv,
                size,
                left,
                right,
                ..
            } => {
                let offset_cmx = cmx + *offset_x;
                let offset_cmv = cmv + *offset_v;
                let cm = offset_cmx + offset_cmv * p1.time;
                // println!("{} = {} + {} * {}", cm, cmx, cmv, p1.time);
                let dx = p1.x - cm;
                let dist_sqr = dx.mag_sq();
                if do_print { println!("dist = {:e}, size = {:e}", dist_sqr, size * size); }
                if size * size < THETA * THETA * dist_sqr {
                    let acc = force.particle_group_accel(i1, p1, &cm, m);
                    if do_print { 
                        println!("group acc {} {} {}", i1, cur_node, acc);
                    }
                    acc
                } else {
                    let left_acc = self.accel_recur(
                        left, i1, p1, spd1, force, particles, ppds, mirror_num, offset_x, offset_v,
                    );
                    let right_acc = self.accel_recur(
                        right, i1, p1, spd1, force, particles, ppds, mirror_num, offset_x, offset_v,
                    );
                    if do_print { 
                        println!("children acc {} {} {}", cur_node, left_acc, right_acc);
                    }
                    left_acc + right_acc
                }
            }
        }
    }

    fn time_recur<F: EventForce>(
        &self,
        cur_node: usize,
        i1: usize,
        p1: &Particle,
        spd1: &F::SingleParticleData,
        force: &F,
        particles: &[Particle],
        accs: &Vec<Vector>,
        mirror_num: usize,
        offset_x: &Vector,
        offset_v: &Vector,
    ) -> f64 {
        // println!("time {}", cur_node);
        match self.nodes[cur_node] {
            KDTree::Leaf {
                num_parts,
                leaf_parts,
            } => {
                let mut time = 1e100;
                for i in 0..(num_parts) {
                    let i2 = leaf_parts[i];
                    if i2 != i1 {
                        let mut p2 = particles[i2].clone();
                        p2.v += *offset_v;
                        p2.x += *offset_x + p2.v * (p1.time - p2.time);
                        let t = force
                            .particle_particle_time_step(i1, p1, i2, &p2, spd1, accs, mirror_num);
                        time = f64::min(t, time);
                        // println!("p:p time {} {} {:e}", i1, i2, time);
                    }
                }
                time
            }
            KDTree::Internal {
                m,
                cmx,
                cmv,
                size,
                left,
                right,
                ..
            } => {
                let offset_cmx = cmx + *offset_x;
                let offset_cmv = cmv + *offset_v;
                let cm = offset_cmx + offset_cmv * p1.time;
                let dx = p1.x - cm;
                let dist_sqr = dx.mag_sq();
                // println!("dist = {}, size = {}", dist, nodes[cur_node].size);
                if size * size < THETA * THETA * dist_sqr {
                    1e100
                } else {
                    let left_time = self.time_recur(
                        left, i1, p1, spd1, force, particles, accs, mirror_num, offset_x, offset_v,
                    );
                    let right_time = self.time_recur(
                        right, i1, p1, spd1, force, particles, accs, mirror_num, offset_x, offset_v,
                    );
                    // println!("Recur time {:e} {:e}", left_time, right_time);
                    f64::min(left_time, right_time)
                }
            }
        }
    }
}

impl Traverser for KDTreeParticleTraverser {
    fn advance_all_on_substep() -> bool {
        false
    }

    fn setup(&mut self, pop: &impl Population) {
        // Build the tree
        let mut indices: Vec<usize> = (0..pop.particles().len()).collect();
        build_tree_par3_chunk(&mut indices, 0, pop.particles(), &mut self.nodes[..], 1);
        // println!("{:?}", self.nodes)
    }

    fn accel_for_one<F: EventForce>(
        &self,
        i1: usize,
        p1: &Particle,
        spd1: &mut F::SingleParticleData,
        force: &F,
        pop: &impl Population,
    ) -> Vector {
        let mut ppds: Vec<(usize, F::ParticlePairData)> = vec![];
        let mut total_acc = Vector::new(0.0, 0.0, 0.0);
        if let Some(mirror_offsets) = pop.boundary_conditions().simple_mirror_offsets() {
            for (mirror_num, (offset_x, offset_v)) in mirror_offsets.iter().enumerate() {
                total_acc += self.accel_recur(
                    0,
                    i1,
                    p1,
                    spd1,
                    force,
                    pop.particles(),
                    &mut ppds,
                    mirror_num,
                    offset_x,
                    offset_v,
                );
            }
        }
        for (i2, ppd) in ppds {
            force.update_particle_pair_data(i1, i2, spd1, Some(ppd));
        }
        total_acc
    }

    fn time_step_for_one<F: EventForce>(
        &self,
        i1: usize,
        p1: &Particle,
        spd1: &F::SingleParticleData,
        force: &F,
        pop: &impl Population,
        accs: &Vec<Vector>,
    ) -> f64 {
        let mut total_time = 1e100;
        if let Some(mirror_offsets) = pop.boundary_conditions().simple_mirror_offsets() {
            for (mirror_num, (offset_x, offset_v)) in mirror_offsets.iter().enumerate() {
                total_time = f64::min(
                    total_time,
                    self.time_recur(
                        0,
                        i1,
                        p1,
                        spd1,
                        force,
                        pop.particles(),
                        accs,
                        mirror_num,
                        offset_x,
                        offset_v,
                    ),
                );
            }
        }
        total_time
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::boundary_conditions::open_boundaries::OpenBoundary;
    use crate::design::basic_population::BasicPopulation;
    use crate::design::system::{self, BoundaryCondition, Particle, Population};
    use crate::forces::kdtree_particle_traverser;
    use crate::forces::kdtree_particle_traverser::KDTreeParticleTraverser;
    use crate::forces::single_particle_event_force::Traverser;
    use crate::vectors::{Axis, Vector};

    #[test]
    fn single_node() {
        let parts = system::two_bodies();
        let bc = OpenBoundary::new();
        let pop = BasicPopulation::new(parts, bc);
        let mut trav = KDTreeParticleTraverser::new(pop.particles().len());
        assert_eq!(trav.nodes.len(), 1);
        trav.setup(&pop);
        match trav.nodes[0] {
            kdtree_particle_traverser::KDTree::Leaf { num_parts, .. } => {
                assert_eq!(num_parts, pop.particles().len())
            }
            _ => assert!(false, "Root isn't leaf of right size when small."),
        };
    }

    #[test]
    fn two_leaves() {
        let parts = system::circular_orbits(11);
        let bc = OpenBoundary::new();
        let pop = BasicPopulation::new(parts, bc);
        let mut trav =
            kdtree_particle_traverser::KDTreeParticleTraverser::new(pop.particles().len());
        trav.setup(&pop);
        recur_test_tree_struct(
            0,
            &trav.nodes,
            pop.particles(),
            Vector::new(-1e100, -1e100, -1e100),
            Vector::new(1e100, 1e100, 1e100),
        );
        assert!(std::matches!(
            trav.nodes[0],
            kdtree_particle_traverser::KDTree::Internal { .. }
        ));
        match (trav.nodes[1], trav.nodes[2]) {
            (
                kdtree_particle_traverser::KDTree::Leaf { num_parts: n1, .. },
                kdtree_particle_traverser::KDTree::Leaf { num_parts: n2, .. },
            ) => {
                assert_eq!(n1 + n2, 12);
            }
            _ => assert!(false, "Node vectors weren't leaves."),
        }
    }

    #[test]
    fn big_solar() {
        let parts = system::circular_orbits(5000);
        let bc = OpenBoundary::new();
        let pop = BasicPopulation::new(parts, bc);
        let mut trav =
            kdtree_particle_traverser::KDTreeParticleTraverser::new(pop.particles().len());
        trav.setup(&pop);
        recur_test_tree_struct(
            0,
            &trav.nodes,
            pop.particles(),
            Vector::new(-1e100, -1e100, -1e100),
            Vector::new(1e100, 1e100, 1e100),
        );
    }

    fn simple_sim<BC: BoundaryCondition>(
        pop: &BasicPopulation<BC>,
        tree: &mut KDTreeParticleTraverser,
        dt: f64,
        steps: i64,
    ) {
        let mut acc = Vec::new();
        for _ in 0..pop.particles().len() {
            acc.push(Vector::new(0.0, 0.0, 0.0))
        }
        // let mut time = Instant::now();
        for _step in 0..steps {
            // if step % 100 == 0 {
            //     let elapsed_secs = time.elapsed().as_nanos() as f64 / 1e9;
            //     println!("Step = {}, duration = {}, n = {}, nodes = {}", step, elapsed_secs, bodies.len(), tree.len());
            //     time = Instant::now();
            // }
            // for i in 0..bodies.len() {
            //     indices[i] = i;
            // }
            // build_tree(&mut indices, 0, bodies.len(), bodies, 0, &mut tree);
            tree.setup(pop);
            // if step % 10 == 0 {
            //     print_tree(step, &tree, &bodies);
            // }
            // acc.iter_mut().enumerate().for_each(|(i, acc)| {
            //   let p1 = pop.particles()[i];
            //   *acc = tree.accel_for_one(i, p1, &bodies, &tree)
            // });

            // pop.particles_mut().iter_mut().zip(&mut acc).for_each(|(b, a)| {
            //     b.v += *a * dt;
            //     b.x += b.v * dt;
            //     *a = Vector::new(0.0, 0.0, 0.0);
            // });
        }
    }

    #[test]
    fn big_solar_with_steps() {
        let mut parts = system::circular_orbits(5000);
        let bc = OpenBoundary::new();
        let mut pop = BasicPopulation::new(parts, bc);
        let mut trav =
            kdtree_particle_traverser::KDTreeParticleTraverser::new(pop.particles().len());
        simple_sim(&pop, &mut trav, 1e-3, 10);
        trav.setup(&pop);
        recur_test_tree_struct(
            0,
            &trav.nodes,
            pop.particles(),
            Vector::new(-1e100, -1e100, -1e100),
            Vector::new(1e100, 1e100, 1e100),
        );
        recur_test_cm_values(0, &trav.nodes, pop.particles());
    }

    fn recur_test_tree_struct(
        node: usize,
        nodes: &Vec<kdtree_particle_traverser::KDTree>,
        particles: &[system::Particle],
        mut min: Vector,
        mut max: Vector,
    ) {
        match nodes[node] {
            kdtree_particle_traverser::KDTree::Leaf {
                num_parts,
                leaf_parts,
            } => {
                for index in 0..num_parts {
                    let i = leaf_parts[index];
                    for dim in Axis::iter() {
                        assert!(
                            particles[i].x[dim] >= min[dim],
                            "Particle dim {:?} is below min. i={} p={} min={}",
                            dim,
                            i,
                            particles[i].x[dim],
                            min[dim]
                        );
                        assert!(
                            particles[i].x[dim] < max[dim],
                            "Particle dim {:?} is above max. i={} p={} max={}",
                            dim,
                            i,
                            particles[i].x[dim],
                            max[dim]
                        );
                    }
                }
            }
            kdtree_particle_traverser::KDTree::Internal {
                split_dim,
                split_val,
                left,
                right,
                ..
            } => {
                let split_dim = split_dim;
                let tmin = min[split_dim];
                let tmax = max[split_dim];
                max[split_dim] = split_val;
                recur_test_tree_struct(left, nodes, particles, min, max);
                max[split_dim] = tmax;
                min[split_dim] = split_val;
                recur_test_tree_struct(right, nodes, particles, min, max);
                min[split_dim] = tmin;
            }
        }
    }

    fn recur_test_cm_values(
        node: usize,
        nodes: &Vec<kdtree_particle_traverser::KDTree>,
        particles: &[system::Particle],
    ) {
      match nodes[node] {
            kdtree_particle_traverser::KDTree::Leaf {
                num_parts,
                leaf_parts,
            } => {
              // No action for a leaf.
            }
            kdtree_particle_traverser::KDTree::Internal {
                m,
                cmx,
                cmv,
                size,
                split_dim,
                split_val,
                left,
                right,
            } => {
                let recur_m = combine_below(node, nodes, particles, &|p| p.m , &|m1, m2| m1 + m2);
                assert_relative_eq!(m, recur_m);
                let recur_cmx = combine_below(node, nodes, particles, &|p| p.x * p.m , &|x1, x2| x1 + x2);
                println!("{} == {} / {}", cmx, recur_cmx, m);
                assert_relative_eq!(cmx.x(), (recur_cmx / m).x(), max_relative = 1e-10);
                assert_relative_eq!(cmx.y(), (recur_cmx / m).y(), max_relative = 1e-10);
                assert_relative_eq!(cmx.z(), (recur_cmx / m).z(), max_relative = 1e-10);
                let recur_cmv = combine_below(node, nodes, particles, &|p| p.v * p.m , &|v1, v2| v1 + v2);
                assert_relative_eq!(cmv.x(), (recur_cmv / m).x(), max_relative = 1e-10);
                assert_relative_eq!(cmv.y(), (recur_cmv / m).y(), max_relative = 1e-10);
                assert_relative_eq!(cmv.z(), (recur_cmv / m).z(), max_relative = 1e-10);
                recur_test_cm_values(left, nodes, particles);
                recur_test_cm_values(right, nodes, particles);
            }
        }
    }

    fn combine_below<E, F: Fn(&Particle) -> E, C: Fn(E, E) -> E>(
        cur_node: usize,
        nodes: &Vec<kdtree_particle_traverser::KDTree>,
        particles: &[Particle],
        f: &F,
        combine: &C,
    ) -> E {
        match nodes[cur_node] {
            kdtree_particle_traverser::KDTree::Leaf {
                num_parts,
                leaf_parts,
            } => {
                let mut sum = f(&particles[leaf_parts[0]]);
                for i in 1..(num_parts) {
                    let i2 = leaf_parts[i];
                    sum = combine(sum, f(&particles[i2]));
                }
                sum
            }
            kdtree_particle_traverser::KDTree::Internal {
                m,
                cmx,
                cmv,
                size,
                left,
                right,
                ..
            } => combine(
                combine_below(left, nodes, particles, f, combine),
                combine_below(right, nodes, particles, f, combine),
            ),
        }
    }
}
