use super::particle::Particle;

pub enum KDTree<'a> {
  InternalNode {
    split_dim: i32,
    split_val: f64,
    m: f64,
    cm: [f64; 3],
    size: f64,
    left: Box<KDTree<'a>>,
    right: Box<KDTree<'a>>
  },
  LeafNode {
    particles: Vec<Particle>
  }
}

pub fn build_tree(particles: &mut Vec<Particle>) -> KDTree {
  // split_at or split_at_mut
  KDTree::LeafNode { particles }
}

pub fn calc_accel(p: Particle, tree: KDTree) -> [f64; 3] {
  [0.0, 0.0, 0.0]
}