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

#[derive(Debug, Default, Clone)]
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
  // pub fn advance(&mut self, dt: f64) {
  //   self.x += self.v * dt;
  //   self.time += dt;
  //   println!("Advance by {} x:{} time:{}", dt, self.x, self.time);
  // }
  pub fn advance_to(&mut self, time: f64) {
    self.x += self.v * (time - self.time);
    self.time = time;
    println!("Advance to {} x:{}", time, self.x);
  }
  pub fn kick(&mut self, dv: &Vector) {
    self.v += *dv;
  }
  // pub fn finish_step(&mut self, dt: f64) {
  //   self.x += self.v * (dt - self.time);
  //   self.time = 0.0;
  // }
}

pub trait Population: Sync + Send {
  type Boundary: BoundaryCondition;
  fn particles(&self) -> &[Particle];
  fn particles_mut(&mut self) -> &mut [Particle];
  fn end_step(&mut self, dt: f64);
  fn apply_boundary_condition(&mut self);
  fn boundary_conditions(&self) -> &Self::Boundary;
}

pub trait Force {
  fn apply_force(&mut self, pop: &mut impl Population);
}

pub struct DoubleForce<F1: Force, F2: Force> {
  f1: F1,
  f2: F2,
}

impl<F1: Force, F2: Force> DoubleForce<F1, F2> {
  pub fn new<FA: Force, FB: Force>(f1: FA, f2: FB) -> DoubleForce<FA, FB>{
    DoubleForce { f1, f2 }
  }
}

impl<F1: Force, F2: Force> Force for DoubleForce<F1, F2> {
  fn apply_force(&mut self, pop: &mut impl Population) {
    self.f1.apply_force(pop);
    self.f2.apply_force(pop);
  }
}

pub trait BoundaryCondition: Sync + Send {
  fn simple_mirror_offsets(&self) -> Option<Vec<(Vector, Vector)>>;
  fn mirrors(&self, p: &Particle) -> impl Iterator<Item = Particle>;
  fn apply(&self, p: &mut Particle);
  fn update(&mut self);
}

pub trait Output {
  fn output<P: Population>(&mut self, step: i64, pop: &P);
}

pub struct System<P: Population, F: Force, Out: Output> {
  pop: P,
  force: F,
  output: Out,
  dt: f64,
  step: i64,
}

impl<P: Population, F: Force, Out: Output> System<P, F, Out> {
  pub fn new(pop: P, force: F, output: Out, dt: f64) -> Self {
    let mut ret = Self {
      pop,
      force,
      output,
      dt,
      step: 0,
    };
    ret.output.output(ret.step, &ret.pop);
    ret
  }

  pub fn kinetic_energy(&self) -> f64 {
    let mut e_k = 0.0;
    for p in self.pop.particles() {
      let v_sqr = p.v.mag_sq();
      e_k += 0.5*v_sqr;
    }
    e_k
  }

  pub fn potential_energy(&self) -> f64 {
    0.0
  }

  pub fn advance(&mut self) {
    self.force.apply_force(&mut self.pop);
    self.pop.end_step(self.dt);
    self.pop.apply_boundary_condition();
    self.step += 1;
    self.output.output(self.step, &self.pop);
    println!("v_k = {:e}", self.kinetic_energy());
  }
}