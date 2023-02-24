use crate::{
    particle::Particle,
    vectors::{Axis, Vector},
};

pub struct HillsForce {
    n: f64,
    kappa: f64,
    n_z: f64,
}

impl HillsForce {
    pub fn new() -> Self {
        Self {
            n: 1.0,
            kappa: 1.0,
            n_z: 1.0,
        }
    }

    pub fn apply_delta_velocity(&self, particles: &mut Vec<Particle>, dt: f64) {
        for p in particles.iter_mut() {
            self.apply_for_one(p, dt);
        }
    }

    fn apply_for_one(&self, p: &mut Particle, dt: f64) {
        let ax = 2. * self.n * p.v.y() - (self.kappa * self.kappa - 4. * self.n * self.n) * p.p.x();
        let ay = -2. * self.n * p.v.x();
        let az = -self.n_z * self.n_z * p.p.z();

        let dv = Vector::new(ax, ay, az) * dt;
        p.v += dv;
    }
}

pub struct SlidingBrickBoundary {
    pub sx: f64,
    pub sy: f64,
}

impl SlidingBrickBoundary {
    pub fn new(sx: f64, sy: f64) -> Self {
        Self { sx, sy }
    }

    pub fn apply(&self, particles: &mut Vec<Particle>, time: f64) {
        for p in particles.iter_mut() {
            self.apply_for_one(p, time);
        }
    }

    fn apply_for_one(&self, p: &mut Particle, _time: f64) {
        let bx = self.sx * 0.5;
        let by = self.sy * 0.5;
        if p.p.y() < -by {
            p.p[Axis::Y] += self.sy;
        } else if p.p.y() > by {
            p.p[Axis::Y] -= self.sy;
        }
        // TODO: fix x for sliding with time (comment copied)
        if p.p.x() < -bx {
            p.p[Axis::X] += self.sx;
            p.v[Axis::Y] -= 1.5 * self.sx;
        } else if p.p.x() > bx {
            p.p[Axis::X] -= self.sx;
            p.v[Axis::Y] += 1.5 * self.sx;
        }
    }
}
