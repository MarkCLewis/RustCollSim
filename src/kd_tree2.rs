use core_simd::*;

use crate::simd_particle::*;

use super::simd_particle::Particle;

const MAX_PARTS: usize = 7;
const THETA: f64 = 0.5;
const NEGS: [usize;MAX_PARTS] = [usize::MAX; MAX_PARTS];

#[derive(Clone, Copy)]
pub struct KDTree {
  // For leaves
  num_parts: usize,
  particles: [usize;MAX_PARTS],

  // For internal nodes
  split_dim: usize,
  split_val: f64,
  m: f64,
  cm: f64x4,
  size: f64,
  left: usize,
  right: usize
}

impl KDTree {
  pub fn leaf<'a>(num_parts: usize, particles: [usize;MAX_PARTS]) -> KDTree {
    KDTree { num_parts: num_parts, particles: particles, 
      split_dim: usize::MAX, split_val: 0.0, m: 0.0, cm: f64x4::splat(0.0), size: 0.0, left: usize::MAX, right: usize::MAX }
  }

  pub fn internal(split_dim: usize, split_val: f64, m: f64, cm: f64x4, size: f64, left: usize, right: usize) -> KDTree {
    KDTree { num_parts: 0, particles: NEGS, 
      split_dim: split_dim, split_val: split_val, m: m, cm: cm, size: size, left: left, right: right }
  }
}

pub fn allocate_node_vec(num_parts: usize) -> Vec<KDTree> {
    let num_nodes = 2 * (num_parts / MAX_PARTS + 1);
    let mut ret = Vec::new();
    ret.resize(num_nodes, KDTree::leaf(0, NEGS));
    ret
}

// Returns the index of the last Node used in the construction.
pub fn build_tree<'a>(indices: &mut Vec<usize>, start: usize, end: usize, particles: &Vec<Particle>, cur_node: usize, nodes: &mut Vec<KDTree>) -> usize {
    // println!("start = {} end = {} cur_node = {}", start, end, cur_node);
    let np = end - start;
    if np <= MAX_PARTS {
        if cur_node >= nodes.len() {
            nodes.resize(cur_node + 1, KDTree::leaf(0, NEGS));
        }
        nodes[cur_node].num_parts = np;
        for i in 0..np {
            nodes[cur_node].particles[i] = indices[start + i]
        }
        cur_node
    } else {
        // Pick split dim and value
        let mut min = f64x4::splat(1e100);
        let mut max = f64x4::splat(-1e100);
        let mut m = 0.0;
        let mut cm = f64x4::splat(0.0);
        for i in 0..np {
            m += particles[indices[i]].m;
            cm += f64x4::splat(particles[indices[i]].m) * particles[indices[i]].p;
            min = f64x4::min(min, particles[indices[i]].p);
            max = f64x4::max(max, particles[indices[i]].p);
        }
        cm /= f64x4::splat(m);
        let mut split_dim = 0;
        for dim in 1..2 {
            if max[dim]-min[dim] > max[split_dim]-min[split_dim] {
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
            indices.swap(s, pivot);
            let mut low = s + 1;
            let mut high = e - 1;
            while low <= high {
                if particles[indices[low]].p[split_dim] < particles[indices[s]].p[split_dim] {
                    low += 1;
                } else {
                    indices.swap(low, high);
                    high -= 1;
                }
            }
            indices.swap(s, high);
            if high < mid {
                s = high + 1;
            } else if high > mid {
                e = high;
            } else {
                s = e;
            }
        }
        let split_val = particles[indices[mid]].p[split_dim];

        // Recurse on children and build this node.
        let left = build_tree(indices, start, mid, particles, cur_node + 1, nodes);
        let right = build_tree(indices, mid, end, particles, left + 1, nodes);

        if cur_node >= nodes.len() {
            nodes.resize(cur_node + 1, KDTree::leaf(0, NEGS));
        }
        nodes[cur_node].num_parts = 0;
        nodes[cur_node].split_dim = split_dim;
        nodes[cur_node].split_val = split_val;
        nodes[cur_node].m = m;
        nodes[cur_node].cm = cm;
        nodes[cur_node].size = size;
        nodes[cur_node].left = left;
        nodes[cur_node].right = right;

        right
    }
}

fn accel_recur(cur_node: usize, p: usize, particles: &Vec<Particle>, nodes: &Vec<KDTree>) -> f64x4 {
  if nodes[cur_node].num_parts > 0 {
    let mut acc = f64x4::splat(0.0);
    for i in 0..(nodes[cur_node].num_parts) {
      if nodes[cur_node].particles[i] != p {
        acc += calc_pp_accel(&particles[p], &particles[nodes[cur_node].particles[i]]);
      }
    }
    acc
  } else {
    let dist = distance(particles[p].p, nodes[cur_node].cm);
    if dist / nodes[cur_node].size < THETA {
      calc_cm_accel(&particles[p], nodes[cur_node].m, nodes[cur_node].cm)
    } else {
      accel_recur(nodes[cur_node].left, p, particles, nodes) + accel_recur(nodes[cur_node].right, p, particles, nodes)
    }
  }
}

pub fn calc_accel(p: usize, particles: &Vec<Particle>, nodes: &Vec<KDTree>) -> f64x4 {
  accel_recur(0, p, particles, nodes)
}

pub fn simple_sim(bodies: &mut Vec<Particle>, dt: f64) {
  let dt_vec = f64x4::splat(dt);
  let mut acc = Vec::new();
  for _ in 0..bodies.len() { 
      acc.push(f64x4::splat(0.0))
  };
  let mut tree = allocate_node_vec(bodies.len());
  let mut indices: Vec<usize> = (0..bodies.len()).collect();
  for step in 1..100001 {
      for i in 0..(bodies.len()) {
        indices[i] = i;
      }
      build_tree(&mut indices, 0, bodies.len(), bodies, 0, &mut tree);
      for i in 0..bodies.len() {
          acc[i] = calc_accel(i, &bodies, &tree);
      }
      for i in 0..bodies.len() { 
          bodies[i].v += dt_vec * acc[i];
          let dp = dt_vec * bodies[i].v;
          bodies[i].p += dp;
          acc[i] = f64x4::splat(0.0);
      }
      if step % 10000 == 0 {
          println!("{} {} {} {} {}", step, bodies[1].p[0], bodies[1].p[1], bodies[1].v[0], bodies[1].v[1]);
      }
  }
}

#[cfg(test)]
mod tests {
    use crate::{simd_particle, kd_tree2};
    use core_simd::*;

    #[test]
    fn single_node() {
        let parts = simd_particle::two_bodies();
        let mut node_vec = kd_tree2::allocate_node_vec(parts.len());
        assert_eq!(node_vec.len(), 2);
        let mut indices: Vec<usize> = (0..parts.len()).collect();
        kd_tree2::build_tree(&mut indices, 0, parts.len(), &parts, 0, &mut node_vec);
        assert_eq!(node_vec[0].num_parts, parts.len());
    }

    fn recur_test_tree_struct(node: usize, nodes: &Vec<kd_tree2::KDTree>, particles: &Vec<simd_particle::Particle>, mut min: f64x4, mut max: f64x4) {
        if nodes[node].num_parts > 0 {
            for index in 0..nodes[node].num_parts {
                let i = nodes[node].particles[index];
                for dim in 0..2 {
                    assert!(particles[i].p[dim] >= min[dim], "Particle dim {} is below min. i={} p={} min={}", dim, i,  particles[i].p[dim], min[dim]);
                    assert!(particles[i].p[dim] < max[dim], "Particle dim {} is above max. i={} p={} max={}", dim, i,  particles[i].p[dim], max[dim]);
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

    #[test]
    fn two_leaves() {
        let parts = simd_particle::circular_orbits(5);
        let mut node_vec = kd_tree2::allocate_node_vec(parts.len());
        assert_eq!(node_vec.len(), 4);
        let mut indices: Vec<usize> = (0..parts.len()).collect();
        kd_tree2::build_tree(&mut indices, 0, parts.len(), &parts, 0, &mut node_vec);
        recur_test_tree_struct(0, &node_vec, &parts, f64x4::splat(-1e100), f64x4::splat(1e100));
        assert_eq!(node_vec[0].num_parts, 0);
        assert_eq!(node_vec[0].split_dim, 0);
        assert_eq!(node_vec[1].num_parts, 5);
        assert_eq!(node_vec[2].num_parts, 6);
    }

    #[test]
    fn big_solar() {
        let parts = simd_particle::circular_orbits(5000);
        let mut node_vec = kd_tree2::allocate_node_vec(parts.len());
        let mut indices: Vec<usize> = (0..parts.len()).collect();
        kd_tree2::build_tree(&mut indices, 0, parts.len(), &parts, 0, &mut node_vec);
        recur_test_tree_struct(0, &node_vec, &parts, f64x4::splat(-1e100), f64x4::splat(1e100));
    }

    #[test]
    fn big_galaxy() {
        let parts = simd_particle::galactic_orbits(50000);
        let mut node_vec = kd_tree2::allocate_node_vec(parts.len());
        let mut indices: Vec<usize> = (0..parts.len()).collect();
        kd_tree2::build_tree(&mut indices, 0, parts.len(), &parts, 0, &mut node_vec);
        recur_test_tree_struct(0, &node_vec, &parts, f64x4::splat(-1e100), f64x4::splat(1e100));
    }
   
}