use crate::no_explode;
use crate::data::basic::Vector;


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
    pub rho: f64
}


impl TestSetup {
    
    pub fn newBasic(v_impact: f64, dt: f64) -> TestSetup {
        TestSetup {
            r1: 1e-7,
            r2: 1e-7,
            v_impact: v_impact,
            integrator: Integrator::Jerk,
            k: no_explode::K,
            b: no_explode::B,
            dt: dt,
            sig_c: 1.0,
            rho: RHO
        }
    }
}


pub struct TestData {
    pub pos: Vec<Vector>,
    pub vel: Vec<Vector>,
    pub rad: Vec<f64>,
    pub acc: Vec<Vector>,
    pub jerk: Vec<Vector>,
    pub phase: CollisionPhase,
    pub entry_rel_vel: f64,
    pub exit_rel_vel: f64,
    pub max_pen_depth: f64,
    pub colliding_steps: i32
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
            entry_rel_vel: f64::NAN,
            exit_rel_vel: f64::NAN,
            max_pen_depth: f64::NAN,
            colliding_steps: 0
        }
    }

    pub fn collisionUpdate(&mut self) {
        assert_eq!(self.pos.len(), 2);
        
        let r = (self.pos[0] - self.pos[1]).mag();
        let delta = r - (self.rad[0] + self.rad[1]);

        if delta < 0. {
            // colliding
            match self.phase {
                CollisionPhase::PreCollision => {
                    let rel_v = (self.vel[1] - self.vel[0]).mag(); // relative vel
                    self.entry_rel_vel = rel_v;
                    self.max_pen_depth = delta;
                    self.colliding_steps += 1;
                    self.phase = CollisionPhase::Colliding;
                },
                CollisionPhase::Colliding => {
                    if delta < self.max_pen_depth {
                        self.max_pen_depth = delta;
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
                let rel_v = (self.vel[1] - self.vel[0]).mag(); // relative vel
                self.exit_rel_vel = rel_v;
                self.phase = CollisionPhase::PostCollision;
            }
        }
    }

    pub fn isDone(&self) -> bool {
        if let CollisionPhase::PostCollision = self.phase { true } else { false }
    }
}