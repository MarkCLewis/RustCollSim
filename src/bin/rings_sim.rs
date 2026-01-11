use rust_coll_sim::boundary_conditions::azimuthal_only::AzimuthalOnly;
use rust_coll_sim::forces::brute_force_particle_traversal::BruteForceParticleTraversal;
use rust_coll_sim::design::coords::{
  CartCoords,
  GCCoords, gc_to_cart
};

use rust_coll_sim::forces::single_particle_event_force::SingleParticleEventForcing;
use rust_coll_sim::forces::gravity_and_soft_sphere_event_force::GravityAndSoftSphereEventForce;
use rust_coll_sim::forces::heap_pq::HeapPQ;
use rust_coll_sim::forces::hills_force::HillsForce;
use rust_coll_sim::boundary_conditions::sliding_brick_boundary::SlidingBrickBoundary;
use rust_coll_sim::design::system::{
  DoubleForce, Output, Particle, Population, System
};

use rust_coll_sim::design::basic_population::BasicPopulation;
use rust_coll_sim::forces::no_explode::{Lewis, Rotter};
use rust_coll_sim::outputs::text_file_output::TextFileOutput;
use rust_coll_sim::vectors::Vector;

use std::f64;

fn main() {
  const NUM_BODIES: usize = 5000;
  let dt = 0.001 * 2.0 * std::f64::consts::PI;
  let sx = 2e-7;
  let sy = 2e-7;
  let rad = 1e-9;
  const CENTRAL_MASS: f64 = 5.683e26; // kg
  const R0: f64 = 1.33e8; // m
  const RHO: f64 = 500.0; // kg/m^3
  let density = RHO * R0 * R0 * R0 / CENTRAL_MASS;
  // let bc = SlidingBrickBoundary::new(sx, sy, dt);
  let bc = AzimuthalOnly::new(sx, sy);
  let mut parts: Vec<Particle> = vec![];
  let mut hard_code = vec![];
  //   Particle {
  //     x: Vector { 0: [0.00000006248691099171291, 0.000000049382848581004685, -0.00000000029173349235852733] },
  //     v: Vector { 0: [-0.0000000020460205993591907, -0.00000009001147643232259, 0.00000000018557434352024934] },
  //     m: 1e-30,
  //     r: 1e-9,
  //     time: 0.0,
  //   },
  //   Particle {
  //     x: Vector { 0: [0.00000005878414265717389, 0.00000004881115610945095, -0.0000000005694926322960055] }, 
  //     v: Vector { 0: [0.00000015845846348985711, -0.00000007533107136245553, -0.000000021646931621800057] },
  //     m: 1e-30,
  //     r: 1e-9,
  //     time: 0.0,
  //   }
  // ];
  fastrand::seed(123);
  while parts.len() < NUM_BODIES {
    let i = parts.len();
    if !hard_code.is_empty() {
      parts.push(hard_code.pop().unwrap());
    } else {
      // let gc = GCCoords {
      //   X: 1e-9 * i as f64,
      //   Y: 2e-9 * i as f64,
      //   e: 0.0,
      //   i: 0.0,
      //   phi: 0.0,
      //   zeta: 0.0
      // };
      let gc = GCCoords {
        X: fastrand::f64() * sx - 0.5 * sx,
        Y: fastrand::f64() * sy - 0.5 * sy,
        e: fastrand::f64() * 1e-8,
        i: fastrand::f64() * 3e-9,
        phi: fastrand::f64() * 2.0 * std::f64::consts::PI,
        zeta: fastrand::f64() * 2.0 * std::f64::consts::PI
      };
      let cc = gc_to_cart(&gc);
      let mut safe = true;
      if cc.p.x() < -sx * 0.5 + rad || cc.p.x() > sx * 0.5 - rad ||
          cc.p.y() < -sx * 0.5 + rad || cc.p.y() > sx * 0.5 - rad {
            safe = false;
      } else {
        for p in parts.iter() {
          if (p.x - cc.p).mag() < p.r + 1.2e-9 {
            safe = false;
            break;
          }
        }
      }
      if safe {
        parts.push(Particle { 
          x: cc.p, 
          v: cc.v, 
          m: 1.3333333 * f64::consts::PI * rad * rad * rad * density, 
          r: rad,
          time: 0.0 });
      }
    }
  }
  type Pop = BasicPopulation<SlidingBrickBoundary>;
  let pop = BasicPopulation::new(parts, bc);
  type Trav<'a> = BruteForceParticleTraversal;
  let traverser = BruteForceParticleTraversal::new();
  type GravEventForce = GravityAndSoftSphereEventForce<Rotter>;
  let spring = Rotter::new(0.5, 0.02);
  let event_force = GravityAndSoftSphereEventForce::new(NUM_BODIES, spring, 20);
  let queue = HeapPQ::new();
  type GravForce<'a> =  SingleParticleEventForcing::<Trav<'a>, GravEventForce, HeapPQ>;
  let grav_coll_force = SingleParticleEventForcing::<Trav<'_>, GravEventForce, HeapPQ>::new(traverser, event_force, queue, dt);
  let hills_force = HillsForce::new(dt);
  let force = DoubleForce::<HillsForce, GravForce>::new(hills_force, grav_coll_force);
  let output = TextFileOutput::new(20, "data.txt");
  let mut sys = System::new(pop, force, output, dt);

  for i in 0..10000 {
    println!("Step {}", i);
    sys.advance();
  }
}
