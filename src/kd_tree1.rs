use super::particle::Particle;

const MAX_PARTS: usize = 7;
const THETA: f64 = 0.5;

pub enum KDTree<'a> {
  InternalNode {
    split_dim: usize,
    split_val: f64,
    m: f64,
    cm: [f64; 3],
    size: f64,
    left: Box<KDTree<'a>>,
    right: Box<KDTree<'a>>
  },
  LeafNode {
    particles: &'a mut [Particle]
  }
}

pub fn build_tree<'a>(particles: &'a mut [Particle]) -> Box<KDTree<'a>> {
  let np = particles.len();
  if particles.len() <= MAX_PARTS {
    Box::new(KDTree::LeafNode { particles })
  } else {
    // Pick split dim and value
    let mut min = [1e100, 1e100, 1e100];
    let mut max = [-1e100, -1e100, -1e100];
    let mut m = 0.0;
    let mut cm = [0.0, 0.0, 0.0];
    for i in 0..np {
      m += particles[i].m;
      for dim in 0 ..2 {
        cm[dim] += particles[i].m * particles[i].p[dim];
        if particles[i].p[dim] < min[dim] {
          min[dim] = particles[i].p[dim];
        }
        if particles[i].p[dim] > max[dim] {
          max[dim] = particles[i].p[dim];
        }
      }
    }
    for dim in 0 ..2 {
      cm[dim] /= m;
    }
    let mut split_dim = 0;
    if max[1]-min[1] > max[0]-min[0] && max[1]-min[1] > max[2]-min[2] {
      split_dim = 1
    } else if max[2]-min[2] > max[0]-min[0] && max[2]-min[2] > max[1]-min[1] {
      split_dim = 2;
    }
    let size = max[split_dim] - min[split_dim];

    // Partition particles on split_dim
    let mid = np / 2;
    let mut start = 0;
    let mut end = np;
    while start + 1 < end {
      let mut low = start + 1;
      let mut high = end - 1;
      while low < high {
        if particles[low].p[split_dim] < particles[start].p[split_dim] {
          low += 1;
        } else {
          particles.swap(low, high);
          high -= 1;
        }
      }
      particles.swap(start, high);
      if high < mid {
        end = high;
      } else if high > mid {
        start = high + 1;
      } else {
        start = end;
      }
    }
    let split_val = particles[mid].p[split_dim];

    // Recurse on children and build this node.
    let (left_parts, right_parts) = particles.split_at_mut(mid);
    let left = build_tree(left_parts);
    let right = build_tree(right_parts);
    Box::new(KDTree::InternalNode {
      split_dim,
      split_val,
      m,
      cm,
      size,
      left: left,
      right: right
    })
  }
}

pub fn calc_accel(p: Particle, tree: Box<KDTree>) -> [f64; 3] {
  let mut accel = [0.0, 0.0, 0.0];

  return accel;
}