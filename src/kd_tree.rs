use std::{cell::RefCell, fs::File, io::Write};

use crate::{particle::*, vectors::Vector};

use super::particle::Particle;

const MAX_PARTS: usize = 7;
const THETA: f64 = 0.3;
const NEGS: [usize; MAX_PARTS] = [usize::MAX; MAX_PARTS];

#[derive(Clone)]
pub struct KDTree {
    pub nodes: Vec<KDTreeNode>,
    pub indices: Vec<usize>,
}

// type PairFuncEmpty = fn(&Particle, &Particle);

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
            let mut min = [1e100, 1e100, 1e100];
            let mut max = [-1e100, -1e100, -1e100];
            let mut m = 0.0;
            let mut cm = [0.0, 0.0, 0.0];
            for i in start..end {
                m += particles[self.indices[i]].m;
                cm[0] += particles[self.indices[i]].m * particles[self.indices[i]].p[0];
                cm[1] += particles[self.indices[i]].m * particles[self.indices[i]].p[1];
                cm[2] += particles[self.indices[i]].m * particles[self.indices[i]].p[2];
                min[0] = f64::min(min[0], particles[self.indices[i]].p[0]);
                min[1] = f64::min(min[1], particles[self.indices[i]].p[1]);
                min[2] = f64::min(min[2], particles[self.indices[i]].p[2]);
                max[0] = f64::max(max[0], particles[self.indices[i]].p[0]);
                max[1] = f64::max(max[1], particles[self.indices[i]].p[1]);
                max[2] = f64::max(max[2], particles[self.indices[i]].p[2]);
            }
            cm[0] /= m;
            cm[1] /= m;
            cm[2] /= m;
            let mut split_dim = 0;
            for dim in 1..3 {
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
                let dx = pop_ref[p].p[0] - self.nodes[cur_node].cm[0];
                let dy = pop_ref[p].p[1] - self.nodes[cur_node].cm[1];
                let dz = pop_ref[p].p[2] - self.nodes[cur_node].cm[2];
                dx * dx + dy * dy + dz * dz
            };
            // println!("dist = {}, size = {}", dist, nodes[cur_node].size);
            if self.nodes[cur_node].size * self.nodes[cur_node].size < THETA * THETA * dist_sqr {
                // particle-node
                pair_func(
                    ParticleIndex(p),
                    Interaction::ParticleNode(
                        Vector(self.nodes[cur_node].cm),
                        self.nodes[cur_node].m,
                    ),
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
    split_dim: usize,
    split_val: f64,
    m: f64,
    cm: [f64; 3],
    size: f64,
    left: usize,
    right: usize,
}

impl KDTreeNode {
    pub fn leaf<'a>(num_parts: usize, particles: [usize; MAX_PARTS]) -> KDTreeNode {
        KDTreeNode {
            num_parts: num_parts,
            particles: particles,
            split_dim: usize::MAX,
            split_val: 0.0,
            m: 0.0,
            cm: [0.0, 0.0, 0.0],
            size: 0.0,
            left: usize::MAX,
            right: usize::MAX,
        }
    }
}

// pub fn simple_sim(bodies: &mut Vec<Particle>, dt: f64, steps: i64) {
//     let mut acc = Vec::new();
//     for _ in 0..bodies.len() {
//         acc.push([0.0, 0.0, 0.0])
//     }
//     // let mut time = Instant::now();
//     let mut tree = allocate_node_vec(bodies.len());
//     let mut indices: Vec<usize> = (0..bodies.len()).collect();
//     for step in 0..steps {
//         // if step % 100 == 0 {
//         //     let elapsed_secs = time.elapsed().as_nanos() as f64 / 1e9;
//         //     println!("Step = {}, duration = {}, n = {}, nodes = {}", step, elapsed_secs, bodies.len(), tree.len());
//         //     time = Instant::now();
//         // }
//         for i in 0..bodies.len() {
//             indices[i] = i;
//         }
//         build_tree(&mut indices, 0, bodies.len(), bodies, 0, &mut tree);
//         // if step % 10 == 0 {
//         //     print_tree(step, &tree, &bodies);
//         // }
//         for i in 0..bodies.len() {
//             acc[i] = calc_accel(i, &bodies, &tree);
//         }
//         for i in 0..bodies.len() {
//             bodies[i].v[0] += dt * acc[i][0];
//             bodies[i].v[1] += dt * acc[i][1];
//             bodies[i].v[2] += dt * acc[i][2];
//             let dx = dt * bodies[i].v[0];
//             let dy = dt * bodies[i].v[1];
//             let dz = dt * bodies[i].v[2];
//             bodies[i].p[0] += dx;
//             bodies[i].p[1] += dy;
//             bodies[i].p[2] += dz;
//             acc[i][0] = 0.0;
//             acc[i][1] = 0.0;
//             acc[i][2] = 0.0;
//         }
//     }
// }

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
                    particles[p].p[0], particles[p].p[1], particles[p].p[2]
                ))?;
            }
        } else {
            file.write_fmt(format_args!(
                "I {} {} {} {}\n",
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
    mut min: [f64; 3],
    mut max: [f64; 3],
) {
    if nodes[node].num_parts > 0 {
        for index in 0..nodes[node].num_parts {
            let i = nodes[node].particles[index];
            for dim in 0..2 {
                assert!(
                    particles[i].p[dim] >= min[dim],
                    "Particle dim {} is below min. i={} p={} min={}",
                    dim,
                    i,
                    particles[i].p[dim],
                    min[dim]
                );
                assert!(
                    particles[i].p[dim] < max[dim],
                    "Particle dim {} is above max. i={} p={} max={}",
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

// #[cfg(test)]
// mod tests {
//     use crate::{kd_tree, particle};

//     #[test]
//     fn single_node() {
//         let parts = particle::two_bodies();
//         let mut node_vec = kd_tree::allocate_node_vec(parts.len());
//         assert_eq!(node_vec.len(), 2);
//         let mut indices: Vec<usize> = (0..parts.len()).collect();
//         kd_tree::build_tree(&mut indices, 0, parts.len(), &parts, 0, &mut node_vec);
//         assert_eq!(node_vec[0].num_parts, parts.len());
//     }

//     #[test]
//     fn two_leaves() {
//         let parts = particle::circular_orbits(11);
//         let mut node_vec = kd_tree::allocate_node_vec(parts.len());
//         assert_eq!(node_vec.len(), 6);
//         let mut indices: Vec<usize> = (0..parts.len()).collect();
//         kd_tree::build_tree(&mut indices, 0, parts.len(), &parts, 0, &mut node_vec);
//         kd_tree::recur_test_tree_struct(
//             0,
//             &node_vec,
//             &parts,
//             [-1e100, -1e100, -1e100],
//             [1e100, 1e100, 1e100],
//         );
//         assert_eq!(node_vec[0].num_parts, 0);
//         assert_eq!(node_vec[1].num_parts + node_vec[2].num_parts, 12);
//     }

//     #[test]
//     fn big_solar() {
//         let parts = particle::circular_orbits(5000);
//         let mut node_vec = kd_tree::allocate_node_vec(parts.len());
//         let mut indices: Vec<usize> = (0..parts.len()).collect();
//         kd_tree::build_tree(&mut indices, 0, parts.len(), &parts, 0, &mut node_vec);
//         kd_tree::recur_test_tree_struct(
//             0,
//             &node_vec,
//             &parts,
//             [-1e100, -1e100, -1e100],
//             [1e100, 1e100, 1e100],
//         );
//     }

//     #[test]
//     fn big_solar_with_steps() {
//         let mut parts = particle::circular_orbits(5000);
//         kd_tree::simple_sim(&mut parts, 1e-3, 10);

//         let mut node_vec = kd_tree::allocate_node_vec(parts.len());
//         let mut indices: Vec<usize> = (0..parts.len()).collect();
//         kd_tree::build_tree(&mut indices, 0, parts.len(), &parts, 0, &mut node_vec);
//         kd_tree::recur_test_tree_struct(
//             0,
//             &node_vec,
//             &parts,
//             [-1e100, -1e100, -1e100],
//             [1e100, 1e100, 1e100],
//         );
//     }
// }
