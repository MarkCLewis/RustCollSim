use std::{cell::RefCell, fs::File, io::Write};

use crate::{
    particle::*,
    vectors::{Axis, Vector},
};

use super::particle::Particle;

const MAX_PARTS: usize = 7;
const THETA: f64 = 0.3;
const NEGS: [usize; MAX_PARTS] = [usize::MAX; MAX_PARTS];

#[derive(Clone)]
pub struct KDTree {
    pub nodes: Vec<KDTreeNode>,
    pub indices: Vec<usize>,
}

#[derive(Debug)]
pub enum Interaction {
    ParticleParticle(ParticleIndex),
    ParticleNode(Vector, f64), // node center of mass loc & mass
}

impl KDTree {
    pub fn new(n: usize) -> Self {
        Self {
            nodes: Self::allocate_node_vec(n),
            indices: (0..n).collect(),
        }
    }

    /// call this to rebuild the tree
    pub fn rebuild_kdtree(&mut self, particles: &Vec<Particle>) {
        self.build_tree(0, particles.len(), particles, 0);
    }

    /// Go through all particles, computing acceleration and giving it to the provided function
    /// the provided function F will be given: (particle index, &particle, interaction (may contain other &particle), acc of p)
    pub fn map_tree<'a, F>(
        &self,
        p: usize,
        particles: &'a RefCell<Vec<Particle>>,
        pair_func: &mut F,
    ) where
        F: FnMut(ParticleIndex, Interaction),
    {
        self.map_tree_recur(0, p, particles, pair_func)
    }

    fn allocate_node_vec(num_parts: usize) -> Vec<KDTreeNode> {
        let num_nodes = 2 * (num_parts / (MAX_PARTS - 1) + 1);
        let mut ret = Vec::new();
        ret.resize(num_nodes, KDTreeNode::leaf(0, NEGS));
        ret
    }

    /// note: this will not catch particles in 2 different leaves moving towards each other
    #[allow(dead_code)]
    pub fn global_relative_speed_estimate_rms(&self, particles: &[Particle]) -> f64 {
        fn recurse(nodes: &[KDTreeNode], particles: &[Particle], node_idx: usize) -> (f64, usize) {
            let node = nodes[node_idx];
            if node.is_leaf() {
                let mut count = 0;

                let mut rel_speed = 0.;

                // this O(n^2) is ok since n is small, like max 7
                for i in 0..node.num_parts {
                    for j in i + 1..node.num_parts {
                        let p1 = particles[node.particles[i]];
                        let p2 = particles[node.particles[j]];

                        rel_speed += (p1.v - p2.v).mag_sq();
                        count += 1;
                    }
                }

                (rel_speed, count)
            } else {
                let (right_rel_speed, right_count) = recurse(nodes, particles, node.right);
                let (left_rel_speed, left_count) = recurse(nodes, particles, node.left);

                (right_rel_speed + left_rel_speed, right_count + left_count)
            }
        }

        let (rel_speed_sum, count) = recurse(&self.nodes, particles, 0);

        (rel_speed_sum / count as f64).sqrt()
    }

    /// note: this will not catch particles in 2 different leaves moving towards each other
    #[allow(dead_code)]
    pub fn global_relative_speed_estimate_max(&self, particles: &[Particle]) -> f64 {
        fn recurse(nodes: &[KDTreeNode], particles: &[Particle], node_idx: usize) -> f64 {
            let node = nodes[node_idx];
            if node.is_leaf() {
                let mut rel_speed: f64 = 0.;

                // this O(n^2) is ok since n is small, like max 7
                for i in 0..node.num_parts {
                    for j in i + 1..node.num_parts {
                        let p1 = particles[node.particles[i]];
                        let p2 = particles[node.particles[j]];

                        rel_speed = rel_speed.max((p1.v - p2.v).mag());
                    }
                }

                rel_speed
            } else {
                let right_rel_speed = recurse(nodes, particles, node.right);
                let left_rel_speed = recurse(nodes, particles, node.left);

                right_rel_speed.max(left_rel_speed)
            }
        }

        recurse(&self.nodes, particles, 0)
    }

    // Returns the index of the last Node used in the construction.
    fn build_tree(
        &mut self,
        start: usize,
        end: usize,
        particles: &Vec<Particle>,
        cur_node: usize,
    ) -> usize {
        // println!("start = {} end = {} cur_node = {}", start, end, cur_node);
        let np = end - start;
        // println!("s = {}, e = {}, cn = {}", start, end, cur_node);
        if np <= MAX_PARTS {
            if cur_node >= self.nodes.len() {
                self.nodes.resize(cur_node + 1, KDTreeNode::leaf(0, NEGS));
            }
            self.nodes[cur_node].num_parts = np;
            for i in 0..np {
                self.nodes[cur_node].particles[i] = self.indices[start + i]
            }
            cur_node
        } else {
            // Pick split dim and value
            let mut min = Vector([1e100, 1e100, 1e100]);
            let mut max = Vector([-1e100, -1e100, -1e100]);
            let mut m = 0.0;
            let mut cm = Vector::ZERO;
            for i in start..end {
                m += particles[self.indices[i]].m;
                cm += particles[self.indices[i]].p * particles[self.indices[i]].m;

                min = min.min_of_every_axis(&particles[self.indices[i]].p);
                max = max.max_of_every_axis(&particles[self.indices[i]].p);
            }
            cm /= m;

            let mut split_dim = Axis::X;
            for dim in Axis::iter() {
                if max[dim] - min[dim] > max[split_dim] - min[split_dim] {
                    split_dim = dim
                }
            }
            let size = max[split_dim] - min[split_dim];

            // Partition particles on split_dim
            let mid = (start + end) / 2;
            let mut s = start;
            let mut e = end;
            while s + 1 < e {
                let pivot = fastrand::usize(s..e);
                self.indices.swap(s, pivot);
                let mut low = s + 1;
                let mut high = e - 1;
                while low <= high {
                    if particles[self.indices[low]].p[split_dim]
                        < particles[self.indices[s]].p[split_dim]
                    {
                        low += 1;
                    } else {
                        self.indices.swap(low, high);
                        high -= 1;
                    }
                }
                self.indices.swap(s, high);
                if high < mid {
                    s = high + 1;
                } else if high > mid {
                    e = high;
                } else {
                    s = e;
                }
            }
            let split_val = particles[self.indices[mid]].p[split_dim];

            // Recurse on children and build this node.
            let left = self.build_tree(start, mid, particles, cur_node + 1);
            let right = self.build_tree(mid, end, particles, left + 1);

            if cur_node >= self.nodes.len() {
                self.nodes.resize(cur_node + 1, KDTreeNode::leaf(0, NEGS));
            }
            self.nodes[cur_node].num_parts = 0;
            self.nodes[cur_node].split_dim = split_dim;
            self.nodes[cur_node].split_val = split_val;
            self.nodes[cur_node].m = m;
            self.nodes[cur_node].cm = cm;
            self.nodes[cur_node].size = size;
            self.nodes[cur_node].left = cur_node + 1;
            self.nodes[cur_node].right = left + 1;

            right
        }
    }

    fn map_tree_recur<'a, 'b, F>(
        &self,
        cur_node: usize,
        p: usize,
        particles: &'a RefCell<Vec<Particle>>,
        pair_func: &mut F,
    ) where
        F: FnMut(ParticleIndex, Interaction),
    {
        if self.nodes[cur_node].num_parts > 0 {
            // do particle-particle
            for i in 0..(self.nodes[cur_node].num_parts) {
                if self.nodes[cur_node].particles[i] > p {
                    // p < self.nodes[cur_node].particles[i] makes sure that
                    // a pair of particle is only fed once to pair_func
                    // and a particle and itself are not fed to pair_func
                    pair_func(
                        ParticleIndex(p),
                        Interaction::ParticleParticle(ParticleIndex(
                            self.nodes[cur_node].particles[i],
                        )),
                    );
                }
            }
        } else {
            let dist_sqr = {
                let pop_ref = &particles.borrow();
                let dx = pop_ref[p].p - self.nodes[cur_node].cm;
                dx * dx
            };
            // println!("dist = {}, size = {}", dist, nodes[cur_node].size);
            if self.nodes[cur_node].size * self.nodes[cur_node].size < THETA * THETA * dist_sqr {
                // particle-node
                pair_func(
                    ParticleIndex(p),
                    Interaction::ParticleNode(self.nodes[cur_node].cm, self.nodes[cur_node].m),
                )
            } else {
                // look into node

                self.map_tree_recur(self.nodes[cur_node].left, p, particles, pair_func);

                self.map_tree_recur(self.nodes[cur_node].right, p, particles, pair_func);
            }
        }
    }
}

#[derive(Clone, Copy)]
pub struct KDTreeNode {
    // For leaves
    num_parts: usize,
    particles: [usize; MAX_PARTS],

    // For internal nodes
    split_dim: Axis,
    split_val: f64,
    m: f64,
    cm: Vector,
    size: f64,
    left: usize,
    right: usize,
}

impl KDTreeNode {
    pub fn leaf<'a>(num_parts: usize, particles: [usize; MAX_PARTS]) -> KDTreeNode {
        KDTreeNode {
            num_parts: num_parts,
            particles: particles,
            split_dim: Axis::X, // or whatever axis
            split_val: 0.0,
            m: 0.0,
            cm: Vector::ZERO,
            size: 0.0,
            left: usize::MAX,
            right: usize::MAX,
        }
    }

    fn is_leaf(&self) -> bool {
        self.num_parts > 0
    }
}

#[allow(dead_code)]
fn print_tree(step: i64, tree: &Vec<KDTreeNode>, particles: &Vec<Particle>) -> std::io::Result<()> {
    let mut file = File::create(format!("tree{}.txt", step))?;

    file.write_fmt(format_args!("{}\n", tree.len()))?;
    for n in tree {
        if n.num_parts > 0 {
            file.write_fmt(format_args!("L {}\n", n.num_parts))?;
            for i in 0..n.num_parts {
                let p = n.particles[i];
                file.write_fmt(format_args!(
                    "{} {} {}\n",
                    particles[p].p.x(),
                    particles[p].p.y(),
                    particles[p].p.z()
                ))?;
            }
        } else {
            file.write_fmt(format_args!(
                "I {:?} {} {} {}\n",
                n.split_dim, n.split_val, n.left, n.right
            ))?;
        }
    }

    Ok(())
}

#[allow(dead_code)]
fn recur_test_tree_struct(
    node: usize,
    nodes: &Vec<KDTreeNode>,
    particles: &Vec<Particle>,
    mut min: Vector,
    mut max: Vector,
) {
    if nodes[node].num_parts > 0 {
        for index in 0..nodes[node].num_parts {
            let i = nodes[node].particles[index];
            for dim in Axis::iter() {
                assert!(
                    particles[i].p[dim] >= min[dim],
                    "Particle dim {:?} is below min. i={} p={} min={}",
                    dim,
                    i,
                    particles[i].p[dim],
                    min[dim]
                );
                assert!(
                    particles[i].p[dim] < max[dim],
                    "Particle dim {:?} is above max. i={} p={} max={}",
                    dim,
                    i,
                    particles[i].p[dim],
                    max[dim]
                );
            }
        }
    } else {
        let split_dim = nodes[node].split_dim;
        let tmin = min[split_dim];
        let tmax = max[split_dim];
        max[split_dim] = nodes[node].split_val;
        recur_test_tree_struct(nodes[node].left, nodes, particles, min, max);
        max[split_dim] = tmax;
        min[split_dim] = nodes[node].split_val;
        recur_test_tree_struct(nodes[node].right, nodes, particles, min, max);
        min[split_dim] = tmin;
    }
}
