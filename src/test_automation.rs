use crate::no_explode;
use crate::data::basic::Vector;
use std::f64::consts::PI;

const RHO: f64 = 0.88;
const DELTA_INIT_FRACTION_OF_RADII: f64 = 0.1;

pub enum Integrator {
    Jerk,
    KickStepKick
}

pub enum CollisionPhase {
    PreCollision,
    Colliding,
    PostCollision
}

pub fn computeMass(r: f64, rho: f64) -> f64 {
    4./3. * r * r * r * PI * rho
}

pub fn calculate_init_vel_for_desired_impact_vel(r0: f64, r1: f64, rho: f64, v_impact: f64, delta: f64) -> f64 {
    let m1 = computeMass(r1, rho);
    let G = 1.;
    (v_impact * v_impact - 2. * G * m1 * delta / ((r0 + r1 + delta) * (r0 + r1))).sqrt()
}

pub struct TestSetup {
    pub r0: f64,
    pub r1: f64,
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
        let (b, k) = no_explode::compute::b_and_k(v_estimate, computeMass(r, RHO));
        TestSetup {
            r0: r,
            r1: r,
            v_impact: v_impact,
            integrator: Integrator::Jerk,
            k: k,
            b: b,
            dt: dt,
            sig_c: 4.0 / r, // 1.0,
            rho: RHO,
            max_time: 0.5 * PI,
            do_graphics: false,
            do_state_dump: false
        }
    }

    pub fn new(v_impact: f64, dt: f64, r0: f64, r1: f64) -> TestSetup {
        let v_estimate = r0.min(r1);
        let mass_avg = (computeMass(r0, RHO) + computeMass(r1, RHO)) / 2.;
        let (b, k) = no_explode::compute::b_and_k(v_estimate, mass_avg);
        let r_avg = (r0 + r1) / 2.;

        TestSetup {
            r0: r0,
            r1: r1,
            v_impact: v_impact,
            integrator: Integrator::Jerk,
            k: k,
            b: b,
            dt: dt,
            sig_c: 4.0 / r_avg,
            rho: RHO,
            max_time: PI,
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

    pub fn newBasic(test: &TestSetup) -> TestData {
        // TODO: change position and velocity
        TestData {
            pos: vec!(Vector(-1.1e-7, 0.0, 0.0), Vector(1.1e-7, 0.0, 0.0)),
            vel: vec!(Vector(0.5e-6, 0.0, 0.0), Vector(-0.5e-6, 0.0, 0.0)),
            rad: vec!(test.r0, test.r1),
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

    pub fn new(test: &TestSetup) -> TestData {
        // DELTA_INIT_FRACTION_OF_RADII
        let multiplier = 1. + DELTA_INIT_FRACTION_OF_RADII / 2.;
        let v0 = calculate_init_vel_for_desired_impact_vel(test.r0, test.r1, test.rho, test.v_impact, DELTA_INIT_FRACTION_OF_RADII * (test.r0 + test.r1));
        TestData {
            pos: vec!(Vector(-test.r0 * multiplier, 0.0, 0.0), Vector(test.r1 * multiplier, 0.0, 0.0)),
            vel: vec!(Vector(v0/2., 0.0, 0.0), Vector(-v0/2., 0.0, 0.0)),
            rad: vec!(test.r0, test.r1),
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


pub struct TestResult {
    coeff_of_res: (f64, f64),
    max_pen_depth_percentage: (f64, f64),
    collision_steps: i32,
    impact_rel_vel: f64,
    time_usage_percent: f64
}

impl TestResult {

    pub fn new(data: &TestData, test: &TestSetup, t: f64) -> TestResult {
        let (enter_vel_0, enter_vel_1) = data.entry_vel;
        let (exit_vel_0, exit_vel_1) = data.exit_vel;

        TestResult {
            coeff_of_res: (exit_vel_0 / enter_vel_0, exit_vel_1 / enter_vel_1),
            max_pen_depth_percentage: (data.max_pen_depth.abs() / test.r0 * 100., data.max_pen_depth.abs() / test.r1 * 100.),
            collision_steps: data.colliding_steps,
            impact_rel_vel: data.rel_impact_vel,
            time_usage_percent: t / test.max_time * 100.
        }
    }

    pub fn print(&self) {
        println!("Results:");
        println!("  Coeff of Res. 0 = {:.3}", self.coeff_of_res.0);
        println!("  Coeff of Res. 1 = {:.3}", self.coeff_of_res.1);
        println!("  Max pen depth 0 = {:.3}%", self.max_pen_depth_percentage.0);
        println!("  Max pen depth 1 = {:.3}%", self.max_pen_depth_percentage.1);
        println!("  Collision Steps = {}", self.collision_steps);
        println!("  Impact velocity = {:.3e}", self.impact_rel_vel);
        println!("  Max time usage  = {:.1}%", self.time_usage_percent);
    }
}