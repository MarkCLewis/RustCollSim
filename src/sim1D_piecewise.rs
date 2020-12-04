#![allow(non_snake_case)]

use std::f64::consts::PI;

use crate::no_explode::compute::b_and_k;

pub struct Particle1D {
    pub x: f64,
    pub v: f64,
    pub mass: f64,
    pub radius: f64,
}

impl Particle1D {
    pub fn new(x: f64, v: f64, mass: f64, radius: f64) -> Self {
        Self { x, v, mass, radius }
    }
}

// GM/(r−radius)2)
// 1/m (−kr)

// −GmM /radius^2

pub struct Simulation {
    pub v0: f64,
    pub k: f64,
    pub b: f64,
}

impl Simulation {
    pub fn new() -> Self {
        Self { v0: 0., k: 0., b: 0. }
    }

    pub fn acc(&mut self, this: &Particle1D, other: &Particle1D) -> f64 {

        let r = (other.x - this.x).abs();
        
        let penDepth = r - (other.radius + this.radius); // pos if sep, neg if colliding
        
        let gravAcc = 1.0 * other.mass / (r * r);

        
        let acc = if penDepth < 0. {
            if self.v0 == 0. {
                self.v0 = (this.v - other.v).abs();
                let (b, k) = b_and_k(self.v0, this.mass);
                self.b = b;
                self.k = k;
                eprintln!("b = {:e} k = {:e}", b,k);
            }

            assert_ne!(self.b, 0.);
            let springAcc = - (self.b / this.mass) * (this.v - other.v).abs() - (self.k / this.mass ) * penDepth;

            -springAcc
        }
        else {
            self.v0 = 0.;
            gravAcc
        };

        if other.x < this.x {
            return -acc;
        }
        return acc;
    }
}

// pub fn testing() {
//     eprintln!("Testing 1");
//     let r: f64 = 1e-7; // 130000 km

//     let rho: f64 = 7.7; //129000; sat mass/ ring radius^3
//     let mass: f64 = 4.0/3.0 * 3.14159 * r * r * r * rho;

//     let p1 = build_Particle1D(0., 0., mass, r);

//     for i in -50..50 {
//         let p2 = build_Particle1D(r * (i as f64) / 10., 0., mass, r);

//         assert_eq!(p2.x, r * (i as f64) / 10.);

//         println!("{}, {}", p2.x , acc(&p2, &p1)); //
//     }
    
// }

pub fn testing2() {
    eprintln!("Testing 2");
    let r = 1e-7; // 1e-7; // 130000 km

    let rho: f64 = 7.7; //129000; sat mass/ ring radius^3
    let mass: f64 = 4.0/3.0 * 3.14159 * r * r * r * rho;

    let h: f64 = 5e-6;//1e-6;

    let mut p1 = Particle1D::new(0., 0., mass, r);
    let mut p2 = Particle1D::new(3. * r, -2.*r, mass, r);//-3e-6

    let mut counter = 0;

    let mut sys = Simulation::new();

    let mut i: f64 = 0.0;
    while i < 0.5 * PI * 2. { 
        let acc1 = sys.acc(&p1, &p2);
        let acc2 = sys.acc(&p2, &p1);
        // kick step
        p1.v += acc1 * h;
        p1.x += p1.v * h;

        p2.v += acc2 * h;
        p2.x += p2.v * h;

        if p1.x.is_nan() {
            eprintln!("Died at {}", i);
            break;
        }

        // if counter > 20 {
        //     break;
        // }

        if counter >= 100 {
            counter = 0;
            println!("{}, {}, {}, {}, {}, {}, {}", i, p1.x, p1.v, acc1, p2.x, p2.v, acc2);
            
        }
        counter += 1;

        i += h;
    }
    println!("");
}