// For parallelism with Rayon, I need to be able to slice up the data.
// One challenge there is the mirrors across bouncary conditions. I don't really want to instantiate
// all the mirrors at one time because of the memory overhead. If we didn't care about proximity
// for things like trees, each particle could be expanded as needed, but mirrors are separated
// and will be in very different parts of any tree. Sliding brick and pure periodic are easy to
// represent as offsets. But other boundary conditions are much more challenging. Handling mirrors
// makes the boundary conditions fundamentally part of the population.
//
// Should I abstract over coordinate types? The extended GC coordinates depended on that in the
// C++ code. At what level should I abstract over that? I could assume everything fundamentally uses
// Cartesian and any population that needs GC could have that be an abstraction. Beyond just GC,
// it is really a type that defines motion and that interacts with boundary conditions.
//
// The slicing needs to be able to assume particles. I think for it to work nicely, I really need
// a full Particle type.

use crate::{design::coords::CartCoords, vectors::Vector};

pub struct Particle {
  pub x: Vector,
  pub v: Vector,
  pub m: f64,
  pub r: f64,
  pub time: f64,
}

impl Particle {
  pub fn new(x: Vector, v: Vector, m: f64, r: f64, time: f64) -> Self {
    Self {
      x,
      v,
      m,
      r,
      time,
    }
  }
}


pub trait Population<BC: BoundaryCondition> {
  fn particles(&self) -> &[Particle];
  fn particles_mut(&mut self) -> &mut [Particle];
  fn end_step(&mut self, dt: f64);
  fn apply_boundary_condition(&mut self);
}

pub trait Force {
  fn apply_force(&self, pop: &mut [Particle]);
}

pub trait BoundaryCondition {
  fn simple_mirror_offsets(&self) -> Option<Vec<Vector>>;
  fn mirrors(&self, p: &Particle) -> impl Iterator<Item = Particle>;
  fn apply(&self, p: &mut Particle);
}

pub trait Output {
  fn output(&self, step: i64);
}

pub struct System<P: Population<BC>, F: Force, BC: BoundaryCondition, Out: Output> {
  pop: P,
  force: F,
  boundary: BC,
  output: Out,
  dt: f64,
  step: i64,
}

impl<P: Population<BC>, F: Force, BC: BoundaryCondition, Out: Output> System<P, F, BC, Out> {
  fn new(pop: P, force: F, boundary: BC, output: Out, dt: f64) -> Self {
    Self {
      pop,
      force,
      boundary,
      output,
      dt,
      step: 0,
    }
  }

  fn advance(&mut self) {
    self.force.apply_force(&mut self.pop.particles_mut());
    self.pop.end_step(self.dt);
    self.pop.apply_boundary_condition();
    self.output.output(self.step);
    self.step += 1;
  }
}