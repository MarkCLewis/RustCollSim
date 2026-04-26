use rust_coll_sim::boundary_conditions::azimuthal_only::AzimuthalOnly;
use rust_coll_sim::design::overlap_grid::generate;
use rust_coll_sim::forces::brute_force_particle_traverser::BruteForceParticleTraverser;
use rust_coll_sim::design::coords::{
  CartCoords,
  GCCoords, gc_to_cart
};

use rust_coll_sim::forces::kdtree_particle_traverser::KDTreeParticleTraverser;
use rust_coll_sim::forces::single_particle_event_force::SingleParticleEventForcing;
use rust_coll_sim::forces::gravity_and_soft_sphere_event_force::GravityAndSoftSphereEventForce;
use rust_coll_sim::forces::heap_pq::HeapPQ;
use rust_coll_sim::forces::hills_force::HillsForce;
use rust_coll_sim::boundary_conditions::sliding_brick_boundary::SlidingBrickBoundary;
use rust_coll_sim::design::system::{
  DoubleForce, NoOutput, Output, Particle, Population, System
};

use rust_coll_sim::design::basic_population::BasicPopulation;
use rust_coll_sim::forces::no_explode::{Lewis, Rotter};
use rust_coll_sim::outputs::text_file_output::TextFileOutput;
use rust_coll_sim::vectors::Vector;

use core::num;
use std::f64;
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
  let size_mult: f64 =   if args.len() > 1 {
    args[1].parse().expect("Argument must be a number.")
  } else {
    1.0
  };
  let num_bodies: usize = (10000.0 * size_mult * size_mult) as usize;
  println!("num_bodies = {}", num_bodies);
  let dt = 0.001 * 2.0 * std::f64::consts::PI;
  let sx = 2e-7 * size_mult;
  let sy = 2e-7 * size_mult;
  let rad = 1e-9;
  const CENTRAL_MASS: f64 = 5.683e26; // kg
  const R0: f64 = 1.33e8; // m
  const RHO: f64 = 500.0; // kg/m^3
  let density = RHO * R0 * R0 * R0 / CENTRAL_MASS;
  let mass = 1.3333333 * f64::consts::PI * rad * rad * rad * density;
  // let bc = SlidingBrickBoundary::new(sx, sy, dt);
  let bc = AzimuthalOnly::new(sy);
  fastrand::seed(123);
  // let mut parts: Vec<Particle> = vec![];
  // while parts.len() < num_bodies {
  //   let i = parts.len();
  //   let gc = GCCoords {
  //     X: fastrand::f64() * sx - 0.5 * sx,
  //     Y: fastrand::f64() * sy - 0.5 * sy,
  //     e: fastrand::f64() * 1e-8,
  //     i: fastrand::f64() * 3e-9,
  //     phi: fastrand::f64() * 2.0 * std::f64::consts::PI,
  //     zeta: fastrand::f64() * 2.0 * std::f64::consts::PI
  //   };
  //   let cc = gc_to_cart(&gc);
  //   let mut safe = true;
  //   if cc.p.x() < -sx * 0.5 + rad || cc.p.x() > sx * 0.5 - rad ||
  //       cc.p.y() < -sx * 0.5 + rad || cc.p.y() > sx * 0.5 - rad {
  //         safe = false;
  //   } else {
  //     for (i, p) in parts.iter().enumerate() {
  //       println!("{} {} {}", cc.p, p.x, (cc.p - p.x).mag());
  //       if (p.x - cc.p).mag() < p.r + rad {
  //         println!("Intersect {i}");
  //         safe = false;
  //         break;
  //       }
  //     }
  //   }
  //   if safe {
  //     println!("Particle at {}", cc.p);
  //     parts.push(Particle { 
  //       x: cc.p, 
  //       v: cc.v, 
  //       m: 1.3333333 * f64::consts::PI * rad * rad * rad * density, 
  //       r: rad,
  //       time: 0.0 });
  //   } else {
  //     println!("Failed for {}", cc.p);
  //   }
  // }

  let particle_gen = || {
    loop {
      let gc = GCCoords {
        X: fastrand::f64() * sx - 0.5 * sx,
        Y: fastrand::f64() * sy - 0.5 * sy,
        e: fastrand::f64() * 1e-8,
        i: fastrand::f64() * 3e-9,
        phi: fastrand::f64() * 2.0 * std::f64::consts::PI,
        zeta: fastrand::f64() * 2.0 * std::f64::consts::PI
      };
      let cc = gc_to_cart(&gc);
      if cc.p.x() >= -sx * 0.5 + rad && cc.p.x() <= sx * 0.5 - rad &&
          cc.p.y() >= -sy * 0.5 + rad && cc.p.y() <= sy * 0.5 - rad {
        return Particle::new(cc.p, cc.v, mass, rad, 0.0);
      }
    }
  };
  let parts = generate(num_bodies, particle_gen, rad);
  println!("Parts generated");

  type Pop = BasicPopulation<SlidingBrickBoundary>;
  let pop = BasicPopulation::new(parts, bc);
  // type Trav<'a> = BruteForceParticleTraverser;
  // let traverser = BruteForceParticleTraverser::new();
  type Trav<'a> = KDTreeParticleTraverser;
  let traverser = KDTreeParticleTraverser::new(num_bodies);
  type GravEventForce = GravityAndSoftSphereEventForce<Rotter>;
  let spring = Rotter::new(0.5, 0.02);
  let event_force = GravityAndSoftSphereEventForce::new(num_bodies, spring, 20);
  let queue = HeapPQ::new();
  type GravForce<'a> =  SingleParticleEventForcing::<Trav<'a>, GravEventForce, HeapPQ>;
  let grav_coll_force = SingleParticleEventForcing::<Trav<'_>, GravEventForce, HeapPQ>::new(traverser, event_force, queue, dt);
  let hills_force = HillsForce::new(dt);
  let force = DoubleForce::<HillsForce, GravForce>::new(hills_force, grav_coll_force);
  let output = NoOutput{}; //TextFileOutput::new(20, "data.txt");
  let mut sys = System::new(pop, force, output, dt);

  for i in 0..1000 {
    println!("Step {}", i);
    sys.advance();
  }
}
