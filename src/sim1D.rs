#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

const b: f64 = 1e10;
const k: f64 = 1e-18;

use crate::data::PI;


pub struct Particle1D {
    pub x: f64,
    pub v: f64,
    pub mass: f64,
    pub radius: f64,
}

pub fn build_Particle1D(x: f64, v: f64, mass: f64, radius: f64) -> Particle1D {
    return Particle1D { x, v, mass, radius };
}

/*
 *
 *      arctan(x)    1
 *  y = --------- + ---
 *          pi       2
 * 
 */
pub fn sigma(x: f64) -> f64 {
    //return x.exp() / (x.exp() + 1.0); // fails if x > 709
    return x.atan() / PI + 0.5;
}

// GM/(r−radius)2)·σ(r·b) +1/m (−kr+p)·σ(−r·b)

// −GmM /radius^2
pub fn acc(this: &Particle1D, other: &Particle1D) -> f64 {
    let p = - 1.0 * this.mass * other.mass / (other.radius * other.radius);

    let r = (other.x - this.x).abs();
    
    let penDepth = r - (other.radius + this.radius); // pos if sep, neg if colliding
    
    let gravAcc = 1.0 * other.mass / (r * r);

    let springAcc = (-k * penDepth + p) / this.mass;

    // actually acc, not force
    let force = springAcc * sigma(-b * penDepth) - gravAcc * sigma(b * penDepth);
    // print everything

    //eprintln!("{} {} {} {}", springAcc, sigma(-b * penDepth), -gravAcc, sigma(b * penDepth));

    if other.x > this.x {
        return -force;
    }
    return force;
}

pub fn testing() {
    eprintln!("Testing 1");
    let r: f64 = 1e-7; // 130000 km

    let rho: f64 = 7.7; //129000; sat mass/ ring radius^3
    let mass: f64 = 4.0/3.0 * 3.14159 * r * r * r * rho;

    let p1 = build_Particle1D(0., 0., mass, r);

    for i in -50..50 {
        let p2 = build_Particle1D(r * (i as f64) / 10., 0., mass, r);

        assert_eq!(p2.x, r * (i as f64) / 10.);

        println!("{}, {}", p2.x , acc(&p2, &p1)); //
    }
    
}

pub fn testing2() {
    eprintln!("Testing 2");
    let r: f64 = 1e-7; // 130000 km

    let rho: f64 = 7.7; //129000; sat mass/ ring radius^3
    let mass: f64 = 4.0/3.0 * 3.14159 * r * r * r * rho;

    let h: f64 = 1e-6;//1e-6;

    let mut p1 = build_Particle1D(0., 0., mass, r);
    let mut p2 = build_Particle1D(5. * r, 0., mass, r);//-3e-6

    let mut counter = 0;

    let mut i: f64 = 0.0;
    while i < 1e-2 * PI * 2. { 
        let acc1 = acc(&p1, &p2);
        let acc2 = acc(&p2, &p1);
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
            println!("{}, {}, {}, {}, {}", i, p1.x, acc1, p2.x, acc2);
            
        }
        counter += 1;

        i += h;
    }
}