#![allow(non_snake_case)]

const b: f64 = 1.0;
const k: f64 = 1.0;

pub struct Particle1D {
    pub x: f64,
    pub mass: f64,
    pub radius: f64,
}

pub fn build_Particle1D(x: f64, mass: f64, radius: f64) -> Particle1D {
    return Particle1D { x, mass, radius };
}

pub fn sigma(x: f64) -> f64 {
    return x.exp() / (x.exp() + 1.0);
}

// GM/(r−radius)2)·σ(r·b) +1/m (−kr+p)·σ(−r·b)

// −GmM /radius^2
pub fn acc(this: &Particle1D, other: &Particle1D) -> f64 {
    let p = - 1.0 * this.mass * other.mass / (other.radius * other.radius);

    let r = (other.x - this.x).abs();
    let penDepth = r - (other.radius + this.radius);
    
    let gravAcc = 1.0 * other.mass / (r * r);
    let springAcc = (-k * (r - (other.radius + this.radius)) + p) / this.mass;

    return gravAcc* sigma(b * penDepth) + springAcc * sigma(-b * penDepth);
}

pub fn testing() {
    let p1 = build_Particle1D(0., 1., 1.);

    for i in 0..100 {
        let p2 = build_Particle1D(3. * (i as f64) / 100.0 + 0.1, 1., 1.);

        println!("{}, {}", p2.x, acc(&p2, &p1));
    }
    
}