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
            match node {
                KDTreeNode::Leaf { num_parts, leaf_parts} => {
                    let mut count = 0;

                    let mut rel_speed = 0.;

                    // this O(n^2) is ok since n is small, like max 7
                    for i in 0..num_parts {
                        for j in i + 1..num_parts {
                            let p1 = &particles[leaf_parts[i]];
                            let p2 = &particles[leaf_parts[j]];

                            rel_speed += (p1.v - p2.v).mag_sq();
                            count += 1;
                        }
                    }

                    (rel_speed, count)
                }
                KDTreeNode::Internal { m, cm, size, left, right, .. } => {
                    let (right_rel_speed, right_count) = recurse(nodes, particles, right);
                    let (left_rel_speed, left_count) = recurse(nodes, particles, left);

                    (right_rel_speed + left_rel_speed, right_count + left_count)
                }
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
            match node {
                KDTreeNode::Leaf { num_parts, leaf_parts} => {
                    let mut rel_speed: f64 = 0.;

                    // this O(n^2) is ok since n is small, like max 7
                    for i in 0..num_parts {
                        for j in i + 1..num_parts {
                            let p1 = particles[leaf_parts[i]];
                            let p2 = particles[leaf_parts[j]];

                            rel_speed = rel_speed.max((p1.v - p2.v).mag());
                        }
                    }

                    rel_speed
                }
                KDTreeNode::Internal { m, cm, size, left, right, .. } => {
                    let right_rel_speed = recurse(nodes, particles, right);
                    let left_rel_speed = recurse(nodes, particles, left);

                    right_rel_speed.max(left_rel_speed)
                }
            }
        }

        recurse(&self.nodes, particles, 0)
    }

    // Returns the index of the last Node used in the construction.
    pub fn build_tree<'a>(
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
            let mut parts = [0; MAX_PARTS];
            for i in 0..np {
                parts[i] = self.indices[start + i]
            }
            self.nodes[cur_node] = KDTreeNode::Leaf { num_parts: np, leaf_parts: parts };
            cur_node
        } else {
            // Pick split dim and value
            let mut min = Vector::new(1e100, 1e100, 1e100);
            let mut max = Vector::new(-1e100, -1e100, -1e100);
            let mut m = 0.0;
            let mut cm = Vector::new(0.0, 0.0, 0.0);
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
            self.nodes[cur_node] = KDTreeNode::Internal { split_dim, split_val, m, cm, size, left: cur_node + 1, right: left+1 };
    
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
        match self.nodes[cur_node] {
            KDTreeNode::Leaf { num_parts, leaf_parts} => {
                // do particle-particle
                for i in 0..num_parts {
                    if leaf_parts[i] > p {
                        // p < self.nodes[cur_node].particles[i] makes sure that
                        // a pair of particle is only fed once to pair_func
                        // and a particle and itself are not fed to pair_func
                        pair_func(
                            ParticleIndex(p),
                            Interaction::ParticleParticle(ParticleIndex(
                                leaf_parts[i],
                            )),
                        );
                    }
                }
            }
            KDTreeNode::Internal { m, cm, size, left, right, .. } => {
                let dist_sqr = {
                    let pop_ref = &particles.borrow();
                    let dx = pop_ref[p].p - cm;
                    dx * dx
                };
                // println!("dist = {}, size = {}", dist, nodes[cur_node].size);
                if size * size < THETA * THETA * dist_sqr {
                    // particle-node
                    pair_func(
                        ParticleIndex(p),
                        Interaction::ParticleNode(cm, m),
                    )
                } else {
                    // look into node

                    self.map_tree_recur(left, p, particles, pair_func);

                    self.map_tree_recur(right, p, particles, pair_func);
                }
            }
        }
    }
}

#[derive(Clone, Copy)]
pub enum KDTreeNode {
    Leaf {
        num_parts: usize,
        leaf_parts: [usize; MAX_PARTS]
    },

    Internal {
        split_dim: Axis,
        split_val: f64,
        m: f64,
        cm: Vector,
        size: f64,
        left: usize,
        right: usize
    }
}

impl KDTreeNode {
    pub fn leaf<'a>(num_parts: usize, particles: [usize; MAX_PARTS]) -> KDTreeNode {
        KDTreeNode::Leaf {
            num_parts: num_parts,
            leaf_parts: particles,
        }
    }
}

#[allow(dead_code)]
fn print_tree(step: i64, tree: &Vec<KDTreeNode>, particles: &Vec<Particle>) -> std::io::Result<()> {
    let mut file = File::create(format!("tree{}.txt", step))?;

    file.write_fmt(format_args!("{}\n", tree.len()))?;
    for n in tree {
        match n {
            KDTreeNode::Leaf { num_parts, leaf_parts } => {
                file.write_fmt(format_args!("L {}\n", num_parts))?;
                for i in 0..*num_parts {
                    let p = leaf_parts[i];
                    file.write_fmt(format_args!(
                        "{} {} {}\n",
                        particles[p].p.x(),
                        particles[p].p.y(),
                        particles[p].p.z()
                    ))?;
                }
            }
            KDTreeNode::Internal { split_dim, split_val, left, right, .. } => {
                file.write_fmt(format_args!(
                    "I {:?} {} {} {}\n",
                    split_dim, split_val, left, right
                ))?;
            }
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
    match nodes[node] {
        KDTreeNode::Leaf { num_parts, leaf_parts } => {
            for index in 0..num_parts {
                let i = leaf_parts[index];
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
        }
        KDTreeNode::Internal { split_dim, split_val, left, right, .. } => {
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
