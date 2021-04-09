use crate::no_explode;
//use crate::data::basic::Vector;
use crate::vectors::Vector;

use std::f64::consts::PI;
use std::fs::File;
use std::io::prelude::*;
use std::path::Path;

/*
 * 
 * v * dt > 2r <- ok passthrough (i.e. expected)
 * if v*dt > 0.1*(r1+r2) skip
 * 
 */

const RHO: f64 = 0.88;
const DELTA_INIT_FRACTION_OF_RADII: f64 = 1e-6;

#[derive(Clone, Copy)]
pub enum Integrator {
    Jerk,
    KickStepKick
}

pub enum CollisionPhase {
    PreCollision,
    Colliding,
    PostCollision
}

#[derive(Clone, Copy)]
pub enum KBCalculator {
    LEWIS,
    ROTTER,
    SCHWARTZ
}

#[derive(Clone, Copy)]
pub enum BlendFunc {
    SIGMOID,
    STEP
}

pub fn computeMass(r: f64, rho: f64) -> f64 {
    4./3. * r * r * r * PI * rho
}

// pub fn calculate_init_vel_for_desired_impact_vel(r0: f64, r1: f64, rho: f64, v_impact: f64, delta: f64) -> f64 {
//     let m1 = computeMass(r1, rho);
//     let G = 1.;
//     (v_impact * v_impact - 2. * G * m1 * delta / ((r0 + r1 + delta) * (r0 + r1))).sqrt()
// }

#[derive(Clone, Copy)]
pub struct TestSetup {
    pub r0: f64,
    pub r1: f64,
    pub v_impact: f64,
    pub integrator: Integrator,
    pub k_b_calc: KBCalculator,
    pub k: f64,
    pub b: f64,
    pub dt: f64,
    //pub sig_c: (f64, f64), // TODO: add w
    pub w: f64,
    pub rho: f64,
    pub max_time: f64,
    pub do_graphics: bool,
    pub do_state_dump: bool,
    pub blend_func: BlendFunc,
    pub sig: fn(f64) -> f64,
    pub sig_dot: fn(f64, f64) -> f64
}


impl TestSetup {
    
    // pub fn newBasic(v_impact: f64, dt: f64) -> TestSetup {
    //     let r = 1e-7; // do not change!! -> b_and_k assumes this value for r
    //     let v_estimate = r;
    //     let (b, k) = no_explode::compute::b_and_k(v_estimate, computeMass(r, RHO));
    //     let w = 1.;
    //     TestSetup {
    //         r0: r,
    //         r1: r,
    //         v_impact: v_impact,
    //         integrator: Integrator::Jerk,
    //         k: k,
    //         b: b,
    //         dt: dt,
    //         sig_c: 4.0 / (w * r), // 1.0,
    //         rho: RHO,
    //         max_time: 0.5 * PI,
    //         do_graphics: false,
    //         do_state_dump: false
    //     }
    // }

    pub fn new(v_impact: f64, dt: f64, r0: f64, r1: f64, w: f64, do_state_dump: bool, k_b_calc: KBCalculator, integrator: Integrator, blend_func: BlendFunc) -> TestSetup {
        let v_estimate = r0.max(r1);
        let m0 = computeMass(r0, RHO);
        let m1 = computeMass(r0, RHO);
        //let mass_avg = (computeMass(r0, RHO) + computeMass(r1, RHO)) / 2.;
        // use reduced mass for b and k
        let reduced_mass = (m0 * m1) / (m0 + m1);
        // use smaller ?
        let (b, k) = match k_b_calc {
            KBCalculator::ROTTER => no_explode::compute::b_and_k3(v_estimate, reduced_mass, r0.min(r1)),
            KBCalculator::LEWIS => no_explode::lewis::b_and_k(v_estimate, reduced_mass, r0.min(r1)),
            KBCalculator::SCHWARTZ => {
                let v_max = v_estimate.max(v_impact);
                no_explode::schwartz::b_and_k(v_max, reduced_mass, r0.min(r1))
            }
        };
        //let r_min = r0.min(r1);

        TestSetup {
            r0: r0,
            r1: r1,
            v_impact: v_impact,
            integrator: integrator,
            k: k.abs(),
            b: b.abs(),
            dt: dt,
            //sig_c: (4.0 / (w * r0), 4.0 / (w * r1)),
            w: w,
            rho: RHO,
            max_time: PI * 10.,
            do_graphics: false,
            do_state_dump: do_state_dump,
            k_b_calc: k_b_calc,
            blend_func: blend_func,
            sig: match blend_func {
                BlendFunc::SIGMOID => crate::fourthOrderInt::sigmoid,
                BlendFunc::STEP => crate::fourthOrderInt::step
            },
            sig_dot: match blend_func {
                BlendFunc::SIGMOID => crate::fourthOrderInt::sigmoidDot,
                BlendFunc::STEP => crate::fourthOrderInt::step_dot
            }
        }
    }

    pub fn print(&self) {
        println!("TestSetup");
        println!("  r0: {:e}, r1: {:e}", self.r0, self.r1);
        println!("  v_impact: {:e}", self.v_impact);
        match self.integrator {
            Integrator::Jerk => println!("  integrator: 4th Order"),
            Integrator::KickStepKick => println!("  integrator: 2nd Order")
        };
        println!("  k: {:e}", self.k);
        println!("  b: {:e}", self.b);
        println!("  dt: {}", self.dt);
        println!("  w: {}", self.w);
        println!("  rho: {}", self.rho);

        println!("  k & b calc: {}", match self.k_b_calc {
            KBCalculator::LEWIS => "Lewis",
            KBCalculator::ROTTER => "Rotter",
            KBCalculator::SCHWARTZ => "SCHWARTZ"
        });
        
    }

    pub fn repr(&self) -> String {
        //new(v_impact: f64, dt: f64, r0: f64, r1: f64, w: f64, do_state_dump: bool, k_b_calc: KBCalculator, integrator: Integrator)
        let calc = match self.k_b_calc {
            KBCalculator::LEWIS => "KBCalculator::LEWIS",
            KBCalculator::ROTTER => "KBCalculator::ROTTER",
            KBCalculator::SCHWARTZ => "KBCalculator::SCHWARTZ"
        };
        let int = match self.integrator {
            Integrator::Jerk => "Integrator::Jerk",
            Integrator::KickStepKick => "Integrator::KickStepKick"
        };
        let blend = match self.blend_func {
            BlendFunc::SIGMOID => "BlendFunc::SIGMOID",
            BlendFunc::STEP => "BlendFunc::STEP"
        };
        format!("TestSetup::new({:e}, {:e}, {:e}, {:e}, {:e}, {}, {}, {}, {})", self.v_impact, self.dt, self.r0, self.r1, self.w, self.do_state_dump, calc, int, blend)
    }
}


pub struct TestData {
    pub pos: Vec<Vector>, // particle 0 should be to the left of particle 1, i.e. have a lower x value
    pub vel: Vec<Vector>,
    pub rad: Vec<f64>,
    pub sig_c: f64,
    pub acc: Vec<Vector>,
    pub jerk: Vec<Vector>,
    pub phase: CollisionPhase,
    pub max_pen_depth: f64,
    pub colliding_steps: i32,
    pub rel_impact_vel: f64,
    pub rel_exit_vel: f64,
    pub setup: TestSetup
}


impl TestData {

    // pub fn newBasic(test: &TestSetup) -> TestData {
    //     // TODO: change position and velocity
    //     TestData {
    //         pos: vec!(Vector(-1.1e-7, 0.0, 0.0), Vector(1.1e-7, 0.0, 0.0)),
    //         vel: vec!(Vector(0.5e-6, 0.0, 0.0), Vector(-0.5e-6, 0.0, 0.0)),
    //         rad: vec!(test.r0, test.r1),
    //         acc: vec!(Vector(0., 0., 0.), Vector(0., 0., 0.)),
    //         jerk: vec!(Vector(0., 0., 0.), Vector(0., 0., 0.)),
    //         phase: CollisionPhase::PreCollision,
    //         entry_vel: (f64::NAN, f64::NAN),
    //         exit_vel: (f64::NAN, f64::NAN),
    //         max_pen_depth: f64::NAN,
    //         colliding_steps: 0,
    //         rel_impact_vel: f64::NAN
    //     }
    // }

    pub fn new(test: &TestSetup) -> TestData {
        // DELTA_INIT_FRACTION_OF_RADII
        let multiplier = 1. + DELTA_INIT_FRACTION_OF_RADII / 2.;
        let v0 = test.v_impact; //calculate_init_vel_for_desired_impact_vel(test.r0, test.r1, test.rho, 
        //    test.v_impact, DELTA_INIT_FRACTION_OF_RADII * (test.r0 + test.r1));
        TestData {
            pos: vec!(Vector(-test.r0 * multiplier, 0.0, 0.0), Vector(test.r1 * multiplier, 0.0, 0.0)),
            vel: vec!(Vector(v0/2., 0.0, 0.0), Vector(-v0/2., 0.0, 0.0)),
            rad: vec!(test.r0, test.r1),
            sig_c: 4. / (test.w * test.r0.min(test.r1)),
            acc: vec!(Vector(0., 0., 0.), Vector(0., 0., 0.)),
            jerk: vec!(Vector(0., 0., 0.), Vector(0., 0., 0.)),
            phase: CollisionPhase::PreCollision,
            max_pen_depth: f64::NAN,
            colliding_steps: 0,
            rel_impact_vel: f64::NAN,
            rel_exit_vel: f64::NAN,
            setup: *test
        }
    }

    pub fn verifyForces(&self) -> Result<(), String> {
        let f0 = self.acc[0] * computeMass(self.rad[0], self.setup.rho);
        let f1 = self.acc[1] * computeMass(self.rad[1], self.setup.rho);
        let diff = (f0 + f1).mag();
        let ratio0 = diff / f0.mag();
        let ratio1 = diff / f1.mag();
        const RATIO: f64 = 0.01;
        if ratio0 > RATIO || ratio1 > RATIO {
            eprintln!("{};", self.setup.repr());
            panic!(format!("test failed - forces unbalanced: {:.2e}, {:.2e}. Ratio is: {:.2e}", f0.mag(), f1.mag(), ratio0.max(ratio1)));
            return Err(format!("test failed - forces unbalanced: {:.2e}, {:.2e}. Ratio is: {:.2e}", f0.mag(), f1.mag(), ratio0.max(ratio1)));
        }

        let j0 = self.acc[0] * computeMass(self.rad[0], self.setup.rho);
        let j1 = self.acc[1] * computeMass(self.rad[1], self.setup.rho);
        let jdiff = (j0 + j1).mag();
        let jratio0 = jdiff / j0.mag();
        let jratio1 = jdiff / j1.mag();
        if jratio0 > RATIO || jratio1 > RATIO {
            //eprintln!("{};", self.setup.repr());
            return Err(format!("test failed - yanks unbalanced: {:.2e}, {:.2e}. Ratio is: {:.2e}", j0.mag(), j1.mag(), jratio0.max(jratio1)));
        }
        return Ok(());
    }

    pub fn interStepCollisionUpdate(&mut self) {
        assert_eq!(self.pos.len(), 2);
        
        let r = (self.pos[0] - self.pos[1]).mag();
        let delta = r - (self.rad[0] + self.rad[1]);
        let pen_depth = delta / 2.;

        if delta < 0. {
            let rel_v = (self.vel[1] - self.vel[0]).mag(); // relative vel

            // colliding
            if let CollisionPhase::PreCollision = self.phase {
                self.rel_impact_vel = rel_v;

                self.max_pen_depth = pen_depth;
                self.phase = CollisionPhase::Colliding;
            }
        }
    }

    pub fn collisionUpdate(&mut self) -> Result<(), String> {
        assert_eq!(self.pos.len(), 2);
        
        let r = (self.pos[0] - self.pos[1]).mag();
        let delta = r - (self.rad[0] + self.rad[1]);
        let pen_depth = delta / 2.;

        // let p = self.vel[0] * mass

        if self.pos[0].0 >= self.pos[1].0 {
            // if particle 0 is to the right of particle 1
            if let CollisionPhase::PreCollision = self.phase {
                // particles too fast and time step too big
                if let Err(msg) = self.requireValidPassThrough() {
                    return Err(msg);
                }
                return Err(String::from("test failed - particles passed through each other without colliding"));
            }
            return Err(format!("test failed - particles passed through each other in {} steps", self.colliding_steps));
        }

        if delta < 0. {
            let rel_v = (self.vel[1] - self.vel[0]).mag(); // relative vel

            // colliding
            match self.phase {
                CollisionPhase::PreCollision => {
                    //self.entry_vel = (self.vel[0].mag(), self.vel[1].mag());
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

                    let rel_acc = (self.acc[1] - self.acc[0]).mag();

                    if rel_v < 1e-12 && rel_acc < 1e-18 {
                        // stuck together
                        self.rel_exit_vel = rel_v;
                        self.phase = CollisionPhase::PostCollision;
                    }
                },
                CollisionPhase::PostCollision => {
                    // whatever
                }
            }
        }
        else {
            // no collision
            match self.phase {
                CollisionPhase::PreCollision => {
                    let rel_pos = (self.pos[1] - self.pos[0]).mag() - (self.rad[1] + self.rad[0]);
                    if rel_pos / (self.rad[1] + self.rad[0]) > 1000. {
                        // the are really far apart, something went wrong
                        return Err(format!("test failed - particles moved far away from each other before colliding, delta: {:e}", rel_pos));
                    }
                }
                CollisionPhase::Colliding => {
                    // end of collision
                    let rel_v = (self.vel[1] - self.vel[0]).mag(); // relative vel
                    //self.exit_vel = (self.vel[0].mag(), self.vel[1].mag());
                    self.rel_exit_vel = rel_v;
                    self.phase = CollisionPhase::PostCollision;
                },
                CollisionPhase::PostCollision => {}
            }
        }

        return Ok(());
    }

    pub fn isDone(&self) -> bool {
        if let CollisionPhase::PostCollision = self.phase { true } else { false }
    }

    pub fn isColliding(&self) -> bool {
        if let CollisionPhase::Colliding = self.phase { true } else { false }
    }

    pub fn requireValidPassThrough(&self) -> Result<(), String> {
        // check if a passthrough is valid
        let v = self.setup.v_impact;
        if v * self.setup.dt > (self.rad[0] + self.rad[1]) {
            // ok
            // this is never the case!
            Ok(())
        }
        else {
            // self.setup.print();

            // println!("Position");
            // self.pos[0].print();
            // self.pos[1].print();
            // println!("Velocities");
            // self.vel[0].print();
            // self.vel[1].print();
            // println!("Radii");
            // println!("{:e}", self.rad[0]);
            // println!("{:e}", self.rad[1]);

            println!("{}", self.setup.repr());
            //panic!("oi");
            Err(format!("Bug! -> this should not happen: v={:e}, dt={:e}, r1+r2={:e}", v, self.setup.dt, self.rad[0] + self.rad[1]))
        }

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
                self.setup.print();
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
    coeff_of_res: f64,
    max_pen_depth_percentage: (f64, f64),
    collision_steps: i32,
    impact_rel_vel: f64,
    time_usage_percent: f64
}

impl TestResult {

    pub fn new(data: &TestData, test: &TestSetup, t: f64) -> TestResult {
        //let (enter_vel_0, enter_vel_1) = data.entry_vel;
        //let (exit_vel_0, exit_vel_1) = data.exit_vel;

        TestResult {
            coeff_of_res: data.rel_exit_vel / data.rel_impact_vel,
            max_pen_depth_percentage: (data.max_pen_depth.abs() / test.r0 * 100., 
                data.max_pen_depth.abs() / test.r1 * 100.),
            collision_steps: data.colliding_steps,
            impact_rel_vel: data.rel_impact_vel,
            time_usage_percent: t / test.max_time * 100.
        }
    }

    pub fn print(&self) {
        println!("Results:");
        println!("  Coeff of Res.   = {:.3}", self.coeff_of_res);
        println!("  Max pen depth 0 = {:.3}%", self.max_pen_depth_percentage.0);
        println!("  Max pen depth 1 = {:.3}%", self.max_pen_depth_percentage.1);
        println!("  Collision Steps = {}", self.collision_steps);
        println!("  Impact velocity = {:.3e}", self.impact_rel_vel);
        println!("  Max time usage  = {:.1}%", self.time_usage_percent);

    }
}

pub struct CSVOutput {
    file: File,
    name: String
}

impl CSVOutput {

    pub fn new(name: &str) -> CSVOutput {
        let path = Path::new(name);
        let display = path.display();

        // Open a file in write-only mode, returns `io::Result<File>`
        let file = match File::create(&path) {
            Err(why) => panic!("couldn't create {}: {}", display, why),
            Ok(file) => file,
        };

        CSVOutput {
            name: String::from(name),
            file: file
        }
    }

    pub fn write(&mut self, s: String) {
        match self.file.write_all(s.as_bytes()) {
            Err(why) => {
                let path = Path::new(&self.name);
                let display = path.display();
                panic!("couldn't write to {}: {}", display, why);
            },
            Ok(_) => {}
        }
    }

    pub fn writeHeader(&mut self) {
        let s = String::from(
            "radius_0,radius_1,desired_impact_vel,integrator,k_c_method,blend_method,k,c,time_step,sigmoid_width,sigmoid_scalar,rho,\
            coeff_of_res,max_pen_depth_percent_0,max_pen_depth_percent_1,\
            collision_steps,real_impact_vel\n");
        self.write(s);
    }

    pub fn writeEntry(&mut self, test: &TestSetup, data: &TestData, result: &TestResult) {
        let integrator = match test.integrator {
            Integrator::Jerk => "4th Order",
            Integrator::KickStepKick => "2nd Order"
        };
        let kb = match test.k_b_calc {
            KBCalculator::LEWIS => "Lewis",
            KBCalculator::ROTTER => "Rotter",
            KBCalculator::SCHWARTZ => "SCHWARTZ"
        };
        let blend = match test.blend_func {
            BlendFunc::SIGMOID => "Sigmoid",
            BlendFunc::STEP => "Step"
        };
        let mut s = format!("{:e},{:e},{:e},{},{},{},{:e},{:e},{:e},{:e},{:e},{:e}", 
            test.r0, test.r1, test.v_impact, integrator, kb, blend, test.k, test.b, test.dt, test.w, data.sig_c, test.rho);
        
        s = format!("{},{:e},{:e},{:e},{},{:e}\n", s, 
            result.coeff_of_res, result.max_pen_depth_percentage.0, 
            result.max_pen_depth_percentage.1, result.collision_steps, result.impact_rel_vel);
        
        self.write(s);
    }

    pub fn writeEntryFailed(&mut self, test: &TestSetup, data: &TestData) {
        let integrator = match test.integrator {
            Integrator::Jerk => "4th Order",
            Integrator::KickStepKick => "2nd Order"
        };
        let kb = match test.k_b_calc {
            KBCalculator::LEWIS => "Lewis",
            KBCalculator::ROTTER => "Rotter",
            KBCalculator::SCHWARTZ => "SCHWARTZ"
        };
        let blend = match test.blend_func {
            BlendFunc::SIGMOID => "Sigmoid",
            BlendFunc::STEP => "Step"
        };
        let mut s = format!("{:e},{:e},{:e},{},{},{},{:e},{:e},{:e},{:e},{:e},{:e}", 
            test.r0, test.r1, test.v_impact, integrator, kb, blend, test.k, test.b, test.dt, test.w, data.sig_c, test.rho);
        
        s = format!("{},,,,,\n", s);

        self.write(s);
    }

}

