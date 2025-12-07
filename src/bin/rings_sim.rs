use rust_coll_sim::design::coords::{
  CartCoords,
  GCCoords, gc_to_cart
};

use rust_coll_sim::design::hills_force::HillsForce;
use rust_coll_sim::design::sliding_brick_boundary::SlidingBrickBoundary;
use rust_coll_sim::design::system::{
  Output, Particle, System, Population
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
  let force = HillsForce::new(dt);
  let mut parts = vec![];
  for i in 0..100 {
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
  let pop = BasicPopulation::new(parts, &bc);
  let output = TextPrintOutput{ interval: 10 };
  let mut sys = System::new(pop, force, output, dt);

  for i in 1..1000 {
    sys.advance();
  }
}
