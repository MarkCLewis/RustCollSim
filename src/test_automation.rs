use crate::no_explode;
use crate::data::basic::Vector;
use std::f64::consts::PI;


const RHO: f64 = 0.88;


pub enum Integrator {
    Jerk,
    KickStepKick
}


pub enum CollisionPhase {
    PreCollision,
    Colliding,
    PostCollision
}


pub struct TestSetup {
    pub r1: f64,
    pub r2: f64,
    pub v_impact: f64,
    pub integrator: Integrator,
    pub k: f64,
    pub b: f64,
    pub dt: f64,
    pub sig_c: f64,
    pub rho: f64,
    pub max_time: f64,
    pub do_graphics: bool,
    pub do_state_dump: bool
}


impl TestSetup {
    
    pub fn newBasic(v_impact: f64, dt: f64) -> TestSetup {
        let r = 1e-7;
        let v_estimate = r;
        let (b, k) = no_explode::compute::b_and_k(v_estimate, 4./3. * r * r * r * PI * RHO);
        TestSetup {
            r1: r,
            r2: r,
            v_impact: v_impact,
            integrator: Integrator::Jerk,
            k: k,
            b: b,
            dt: dt,
            sig_c: 1.0,
            rho: RHO,
            max_time: 0.5 * PI,
            do_graphics: false,
            do_state_dump: false
        }
    }
}


pub struct TestData {
    pub pos: Vec<Vector>, // particle 0 should be to the left of particle 1, i.e. have a lower x value
    pub vel: Vec<Vector>,
    pub rad: Vec<f64>,
    pub acc: Vec<Vector>,
    pub jerk: Vec<Vector>,
    pub phase: CollisionPhase,
    pub entry_vel: (f64, f64),
    pub exit_vel: (f64, f64),
    pub max_pen_depth: f64,
    pub colliding_steps: i32,
    pub rel_impact_vel: f64
}


impl TestData {

    pub fn new(test: &TestSetup) -> TestData {
        // TODO: change position and velocity
        TestData {
            pos: vec!(Vector(-1.1e-7, 0.0, 0.0), Vector(1.1e-7, 0.0, 0.0)),
            vel: vec!(Vector(0.5e-6, 0.0, 0.0), Vector(-0.5e-6, 0.0, 0.0)),
            rad: vec!(test.r1, test.r2),
            acc: vec!(Vector(0., 0., 0.), Vector(0., 0., 0.)),
            jerk: vec!(Vector(0., 0., 0.), Vector(0., 0., 0.)),
            phase: CollisionPhase::PreCollision,
            entry_vel: (f64::NAN, f64::NAN),
            exit_vel: (f64::NAN, f64::NAN),
            max_pen_depth: f64::NAN,
            colliding_steps: 0,
            rel_impact_vel: f64::NAN
        }
    }

    pub fn collisionUpdate(&mut self) {
        assert_eq!(self.pos.len(), 2);
        
        let r = (self.pos[0] - self.pos[1]).mag();
        let delta = r - (self.rad[0] + self.rad[1]);
        let pen_depth = delta / 2.;

        if self.pos[0].0 >= self.pos[1].0 {
            // if particle 0 is to the right of particle 1
            panic!("test failed - particles passed through each other");
        }

        if delta < 0. {
            // colliding
            match self.phase {
                CollisionPhase::PreCollision => {
                    let rel_v = (self.vel[1] - self.vel[0]).mag(); // relative vel
                    self.entry_vel = (self.vel[0].mag(), self.vel[1].mag());
                    self.rel_impact_vel = rel_v;

                    self.max_pen_depth = pen_depth;
                    self.colliding_steps += 1;
                    self.phase = CollisionPhase::Colliding;
                },
                CollisionPhase::Colliding => {
                    if pen_depth < self.max_pen_depth {
                        self.max_pen_depth = pen_depth;
                    }
                    self.colliding_steps += 1;
                },
                CollisionPhase::PostCollision => {
                    // whatever
                }
            }
        }
        else {
            // no collision
            if let CollisionPhase::Colliding = self.phase {
                // end of collision
                // let rel_v = (self.vel[1] - self.vel[0]).mag(); // relative vel
                self.exit_vel = (self.vel[0].mag(), self.vel[1].mag());
                self.phase = CollisionPhase::PostCollision;
            }
        }
    }

    pub fn isDone(&self) -> bool {
        if let CollisionPhase::PostCollision = self.phase { true } else { false }
    }

    pub fn requireDone(&self) {
        if !self.isDone() {
            panic!("test failed - no collision");
        }
    }

    pub fn requireFinite(&self) {
        for p in &self.pos {
            if !p.is_finite() {
                p.print();
                panic!("test failed - got non-finite position");
            }
        }
        for v in &self.vel {
            if !v.is_finite() {
                v.print();
                panic!("test failed - got non-finite velocity");
            }
        }
    }
}