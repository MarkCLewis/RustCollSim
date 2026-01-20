use crate::{design::system::{BoundaryCondition, Particle}, vectors::{Axis, Vector}};

pub struct AzimuthalOnly {
    pub sy: f64,
}

impl AzimuthalOnly {
    pub fn new(sy: f64) -> Self {
        Self { sy }
    }

    fn apply_for_one(&self, p: &mut Particle) {
        let by = self.sy * 0.5;
        if p.x.y() < -by {
            p.x[Axis::Y] += self.sy;
        } else if p.x.y() > by {
            p.x[Axis::Y] -= self.sy;
        }
    }
}

impl BoundaryCondition for AzimuthalOnly {
    fn simple_mirror_offsets(&self) -> Option<Vec<(Vector, Vector)>> {
      Some(vec![
        (Vector::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 0.0)),
        (Vector::new(0.0, self.sy, 0.0), Vector::new(0.0, 0.0, 0.0)),
        (Vector::new(0.0, -self.sy, 0.0), Vector::new(0.0, 0.0, 0.0)),
      ])
    }

    fn mirrors(&self, p: &Particle) -> impl Iterator<Item = Particle> {
      AzimuthalOnlyIterator::new(p, self)
    }

    fn apply(&self, p: &mut Particle) {
        self.apply_for_one(p);
    }

    fn update(&mut self) {
    }
}

struct AzimuthalOnlyIterator<'a, 'b> {
  particle: &'a Particle,
  boundary: &'b AzimuthalOnly,
  state: u8,
}

impl<'a, 'b> AzimuthalOnlyIterator<'a, 'b> {
  fn new(p: &'a Particle, boundary: &'b AzimuthalOnly) -> Self {
    Self {
      particle: p,
      boundary,
      state: 0,
    }
  }
}

impl<'a, 'b> Iterator for AzimuthalOnlyIterator<'a, 'b> {
  type Item = Particle;

  fn next(&mut self) -> Option<Self::Item> {
    self.state += 1;
    let p = self.particle;

    return match self.state {
      1 => {
        // Original
        Some(
          Particle::new(
            p.x,
            p.v,
            p.m,
            p.r,
            p.time,
          )
        )
      }
      1 => {
        // Mirror in Y
        Some(
          Particle::new(
            p.x + Vector::new(0.0, self.boundary.sy, 0.0),
            p.v,
            p.m,
            p.r,
            p.time,
          )
        )
      }
      2 => {
        // Mirror in -Y
        Some(
          Particle::new(
            p.x - Vector::new(0.0, self.boundary.sy, 0.0),
            p.v,
            p.m,
            p.r,
            p.time,
          )
        )
      }
      _ => None
    }
  }
}