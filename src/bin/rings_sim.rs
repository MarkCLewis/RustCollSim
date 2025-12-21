use rust_coll_sim::design::brute_force_particle_traversal::BruteForceParticleTraversal;
use rust_coll_sim::design::coords::{
  CartCoords,
  GCCoords, gc_to_cart
};

use rust_coll_sim::design::event_force::SingleParticleEventForcing;
use rust_coll_sim::design::gravity_and_soft_sphere_event_force::GravityAndSoftSphereEventForce;
use rust_coll_sim::design::heap_pq::HeapPQ;
use rust_coll_sim::design::hills_force::HillsForce;
use rust_coll_sim::design::sliding_brick_boundary::SlidingBrickBoundary;
use rust_coll_sim::design::system::{
  DoubleForce, Output, Particle, Population, System
};

use rust_coll_sim::design::basic_population::BasicPopulation;
use rust_coll_sim::vectors::Vector;

struct TextPrintOutput {
  interval: i64,
}

impl Output for TextPrintOutput {
    fn output<P: Population>(&self, step: i64, pop: &P) {
      if step % self.interval == 0 {
        for (i, Particle { x, v, m, r, time }) in pop.particles().iter().enumerate() {
          println!("{} {} {} {} {} {} {} {} {} {} {}",step, i, x.x(), x.y(), x.z(), v.x(), v.y(), v.z(), m, r, time);
        }
      }
    }
}


fn main() {
  let dt = 0.001 * 2.0 * std::f64::consts::PI;
  let sx = 2e-7;
  let sy = 2e-7;
  let bc = SlidingBrickBoundary::new(sx, sy);
  const NUM_BODIES: usize = 100;
  let mut parts = vec![];
  for i in 0..NUM_BODIES {
    let gc = GCCoords {
      X: fastrand::f64() * sx - 0.5 * sx,
      Y: fastrand::f64() * sy - 0.5 * sy,
      e: fastrand::f64() * 1e-8,
      i: fastrand::f64() * 1e-9,
      phi: fastrand::f64() * 2.0 * std::f64::consts::PI,
      zeta: fastrand::f64() * 2.0 * std::f64::consts::PI
    };
    let cc = gc_to_cart(&gc);
    parts.push(Particle { 
      x: cc.p, 
      v: cc.v, 
      m: 1e-30, 
      r: 1e-8, 
      time: 0.0 });
  }
  type Pop<'a> = BasicPopulation<'a, SlidingBrickBoundary>;
  let mut pop = BasicPopulation::new(parts, &bc);
  type Trav<'a> = BruteForceParticleTraversal;
  let traverser = BruteForceParticleTraversal::new();
  type GravEventForce = GravityAndSoftSphereEventForce;
  let event_force = GravityAndSoftSphereEventForce::new(NUM_BODIES);
  let queue = HeapPQ::new();
  type GravForce<'a> =  SingleParticleEventForcing::<Trav<'a>, GravEventForce, HeapPQ>;
  let grav_coll_force = SingleParticleEventForcing::<Trav<'_>, GravEventForce, HeapPQ>::new(traverser, event_force, queue, dt);
  let hills_force = HillsForce::new(dt);
  let force = DoubleForce::<HillsForce, GravForce>::new(hills_force, grav_coll_force);
  let output = TextPrintOutput{ interval: 10 };
  let mut sys = System::new(pop, force, output, dt);

  for i in 1..1000 {
    sys.advance();
  }
}
