use crate::vectors::Vector;

const A0: f64 = 0.75;
const BETA: f64 = 2.0;

pub struct CartCoords {
  pub p: Vector,
  pub v: Vector
}

pub struct GCCoords {
  pub X: f64,
  pub Y: f64,
  pub e: f64,
  pub i: f64,
  pub phi: f64,
  pub zeta: f64
}

pub fn cart_to_gc(cc: &CartCoords) -> GCCoords {
  let X = (2.0*cc.p.x()+cc.v.y())*2.0;
  let Y = cc.p.y()-cc.v.x()*BETA;
  let dx = X-cc.p.x();
  let dy = cc.p.y()-Y;
  let zeta = f64::atan2(-cc.v.z(),cc.p.z());
  GCCoords {
    X,
    Y,
    e: f64::sqrt(dx*dx+dy*dy/(BETA*BETA)),
    i: f64::abs(cc.p.z()/f64::cos(zeta)),
    phi: f64::atan2(cc.p.y()-Y,BETA*(X-cc.p.x())),
    zeta,
  }
}

pub fn gc_to_cart(gc: &GCCoords) -> CartCoords {
  CartCoords {
    p: Vector ([
      gc.X - gc.e * f64::cos(gc.phi),
      gc.Y + BETA * gc.e * f64::sin(gc.phi),
      gc.i * f64::cos(gc.zeta)
    ] ),
    v: Vector ( [
      gc.e * f64::sin(gc.phi),
      BETA * gc.e * f64::cos(gc.phi) - 2.0 * A0 * gc.X,
      gc.i * f64::sin(gc.zeta)
    ] )
  }
}