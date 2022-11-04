use crate::particle::Particle;

pub trait ShortRangeForce {
    fn calc_force(p1: &Particle, p2: &Particle) -> [f64; 3];
}
