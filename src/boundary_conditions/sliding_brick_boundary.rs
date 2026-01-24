use crate::{design::system::{BoundaryCondition, Particle}, vectors::{Axis, Vector}};

pub struct SlidingBrickBoundary {
    pub sx: f64,
    pub sy: f64,
    y_offset: f64,
    dt: f64,
}

impl SlidingBrickBoundary {
    pub fn new(sx: f64, sy: f64, dt: f64) -> Self {
        Self { sx, sy, y_offset: 0.0 , dt }
    }

    fn apply_for_one(&self, p: &mut Particle) {
        let bx = self.sx * 0.5;
        let by = self.sy * 0.5;
        if p.x.y() < -by {
            p.x[Axis::Y] += self.sy;
        } else if p.x.y() > by {
            p.x[Axis::Y] -= self.sy;
        }
        if p.x.x() < -bx {
            p.x[Axis::X] += self.sx;
            p.x[Axis::Y] += self.y_offset;
            while p.x[Axis::Y] > by {
              p.x[Axis::Y] -= self.sy;
            }
            while p.x[Axis::Y] < -by {
              p.x[Axis::Y] += self.sy;
            }
            p.v[Axis::Y] -= 1.5 * self.sx;
        } else if p.x.x() > bx {
            p.x[Axis::X] -= self.sx;
            p.x[Axis::Y] -= self.y_offset;
            while p.x[Axis::Y] > by {
              p.x[Axis::Y] -= self.sy;
            }
            while p.x[Axis::Y] < -by {
              p.x[Axis::Y] += self.sy;
            }
            p.v[Axis::Y] += 1.5 * self.sx;
        }
    }

    fn base_y_offset(&self) -> f64 {
      if self.y_offset > 0.0 {
        self.y_offset - 2.0 * self.sy
      } else {
        self.y_offset - self.sy
      }
    }
}

impl BoundaryCondition for SlidingBrickBoundary {
    fn simple_mirror_offsets(&self) -> Option<Vec<(Vector, Vector)>> {
      let base_y_offset = self.base_y_offset();
      Some(vec![
        (Vector::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 0.0)),
        (Vector::new(0.0, self.sy, 0.0), Vector::new(0.0, 0.0, 0.0)),
        (Vector::new(0.0, -self.sy, 0.0), Vector::new(0.0, 0.0, 0.0)),
        (Vector::new(self.sx, base_y_offset, 0.0), Vector::new(0.0, -1.5 * self.sx, 0.0)),
        (Vector::new(self.sx, base_y_offset + self.sy, 0.0), Vector::new(0.0, -1.5 * self.sx, 0.0)),
        (Vector::new(self.sx, base_y_offset + 2.0 * self.sy, 0.0), Vector::new(0.0, -1.5 * self.sx, 0.0)),
        (Vector::new(-self.sx, -base_y_offset, 0.0), Vector::new(0.0, 1.5 * self.sx, 0.0)),
        (Vector::new(-self.sx, -base_y_offset - self.sy, 0.0), Vector::new(0.0, 1.5 * self.sx, 0.0)),
        (Vector::new(-self.sx, -base_y_offset - 2.0 * self.sy, 0.0), Vector::new(0.0, 1.5 * self.sx, 0.0)),
      ])
    }

    fn mirrors(&self, p: &Particle) -> impl Iterator<Item = Particle> {
      SlidingBrickIterator::new(p, self)
    }

    fn apply(&self, p: &mut Particle) {
        self.apply_for_one(p);
    }

    fn update(&mut self) {
      self.y_offset -= 1.5 * self.sx * self.dt;
      while self.y_offset < -0.5 * self.sy {
        self.y_offset += self.sy;
      }
      println!("y_offset for sliding brick is {:e}", self.y_offset);
    }
}

struct SlidingBrickIterator<'a, 'b> {
  particle: &'a Particle,
  boundary: &'b SlidingBrickBoundary,
  base_y_offset: f64,
  state: u8,
}

impl<'a, 'b> SlidingBrickIterator<'a, 'b> {
  fn new(p: &'a Particle, boundary: &'b SlidingBrickBoundary) -> Self {
    let base_y_offset = boundary.base_y_offset();
    Self {
      particle: p,
      boundary,
      base_y_offset,
      state: 0,
    }
  }
}

impl<'a, 'b> Iterator for SlidingBrickIterator<'a, 'b> {
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
      3 => {
        // First mirror in X
        Some(
          Particle::new(
            p.x + Vector::new(self.boundary.sx, self.base_y_offset, 0.0),
            p.v - Vector::new(0.0, 1.5 * self.boundary.sx, 0.0),
            p.m,
            p.r,
            p.time,
          )
        )
      }
      4 => {
        // Second mirror in X
        Some(
          Particle::new(
            p.x + Vector::new(self.boundary.sx, self.base_y_offset + self.boundary.sy, 0.0),
            p.v - Vector::new(0.0, 1.5 * self.boundary.sx, 0.0),
            p.m,
            p.r,
            p.time,
          )
        )
      }
      5 => {
        // Third mirror in X
        Some(
          Particle::new(
            p.x + Vector::new(self.boundary.sx, self.base_y_offset + 2.0 * self.boundary.sy, 0.0),
            p.v - Vector::new(0.0, 1.5 * self.boundary.sx, 0.0),
            p.m,
            p.r,
            p.time,
          )
        )
      }
      6 => {
        // First mirror in -X
        Some(
          Particle::new(
            p.x - Vector::new(self.boundary.sx, self.base_y_offset, 0.0),
            p.v + Vector::new(0.0, 1.5 * self.boundary.sx, 0.0),
            p.m,
            p.r,
            p.time,
          )
        )
      }
      7 => {
        // Second mirror in -X
        Some(
          Particle::new(
            p.x - Vector::new(self.boundary.sx, self.base_y_offset - self.boundary.sy, 0.0),
            p.v + Vector::new(0.0, 1.5 * self.boundary.sx, 0.0),
            p.m,
            p.r,
            p.time,
          )
        )
      }
      8 => {
        // Third mirror in -X
        Some(
          Particle::new(
            p.x - Vector::new(self.boundary.sx, self.base_y_offset - 2.0 * self.boundary.sy, 0.0),
            p.v + Vector::new(0.0, 1.5 * self.boundary.sx, 0.0),
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