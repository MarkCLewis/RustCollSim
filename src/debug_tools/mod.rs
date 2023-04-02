use std::cell::RefCell;

pub mod overlap;

use crate::{particle::Particle, vectors::Vector};

pub trait ChangeInspector {
    fn before(data: &RefCell<Vec<Particle>>, change_trigger_percent: f64) -> Self;
    fn after(&mut self, data: &RefCell<Vec<Particle>>);
}

pub struct NoneInspector {}

impl ChangeInspector for NoneInspector {
    fn before(_: &RefCell<Vec<Particle>>, _: f64) -> Self {
        Self {}
    }

    fn after(&mut self, _: &RefCell<Vec<Particle>>) {}
}

pub struct VelocityInspector {
    before: Vec<Vector>,
    change_trigger_percent: f64,
}

impl ChangeInspector for VelocityInspector {
    fn before(data: &RefCell<Vec<Particle>>, change_trigger_percent: f64) -> Self {
        let before = data.borrow().iter().map(|p| p.v).collect::<Vec<Vector>>();
        Self {
            before,
            change_trigger_percent,
        }
    }

    fn after(&mut self, data: &RefCell<Vec<Particle>>) {
        let after = data.borrow().iter().map(|p| p.v).collect::<Vec<Vector>>();
        for (i, (before, after)) in self.before.iter().zip(after.iter()).enumerate() {
            let percent_change = (*after - *before).mag() / before.mag() * 100.;

            if percent_change > self.change_trigger_percent {
                println!(
                    "Velocity of particle {} changed {:.2}%, from {:?} to {:?}",
                    i, percent_change, before, after
                );
            }
        }
    }
}
