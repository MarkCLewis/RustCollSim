use crate::{design::system::{BoundaryCondition, Particle}, vectors::Vector};

pub struct OpenBoundary {}

impl OpenBoundary {
    pub fn new() -> OpenBoundary {
        OpenBoundary {  }
    }
}

impl BoundaryCondition for OpenBoundary {

    fn simple_mirror_offsets(&self) -> Option<Vec<(Vector, Vector)>> {
        Some(vec![(Vector::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 0.0))])
    }

    fn mirrors(&self, p: &Particle) -> impl Iterator<Item = Particle> {
        std::iter::once(p.clone())
    }

    fn apply(&self, p: &mut crate::design::system::Particle) {
    }

    fn update(&mut self) {
    }
}