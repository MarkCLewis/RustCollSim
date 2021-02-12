

pub const PI: f64 = 3.141592653589793;
pub const K_DEFAULT: f64 =  1e11;
pub const DRAG_DEFAULT: f64 = 1e3;

pub mod basic {
    use std::ops::{Mul, Div, Sub, Add, AddAssign, SubAssign};

    fn square(x: f64) -> f64 { x * x }

    // x, y, z for all
    #[derive(Clone, Copy)]
    pub struct Vector(pub f64, pub f64, pub f64);

    impl Vector {
        pub fn dot(&self, other: &Vector) -> f64 {
            self.0 * other.0 + self.1 * other.1 + self.2 + other.2
        }

        pub fn print(&self) {
            println!("<{:e}, {:e}, {:e}>", self.0, self.1, self.2);
        }

        pub fn is_finite(&self) -> bool {
            self.0.is_finite() && self.1.is_finite() && self.2.is_finite()
        }

        pub fn mag(&self) -> f64 {
            (square(self.0) + square(self.1) + square(self.2)).sqrt()
        }

        pub fn unit_vector(&self) -> Vector {
            let len = self.mag();
            Vector(self.0 / len, self.1 / len, self.2 / len)
        }
    }

    impl Add for Vector {
        type Output = Self;
        fn add(self, other: Self) -> Self {
            Self(self.0 + other.0, self.1 + other.1, self.2 + other.2)
        }
    }

    impl AddAssign for Vector {
        fn add_assign(&mut self, other: Self) {
            *self = Self(self.0 + other.0, self.1 + other.1, self.2 + other.2);
        }
    }

    impl Sub for Vector {
        type Output = Self;
        fn sub(self, other: Self) -> Self {
            Self(self.0 - other.0, self.1 - other.1, self.2 - other.2)
        }
    }

    impl SubAssign for Vector {
        fn sub_assign(&mut self, other: Self) {
            *self = Self(self.0 - other.0, self.1 - other.1, self.2 - other.2);
        }
    }

    impl Div<f64> for Vector {
        type Output = Self;

        fn div(self, f: f64) -> Self {
            Self(self.0/f, self.1/f, self.2/f)
        }
    }

    impl Mul<f64> for Vector {
        type Output = Self;

        fn mul(self, f: f64) -> Self {
            Self(self.0*f, self.1*f, self.2*f)
        }
    }

    // x, y, z for all
    #[derive(Clone, Copy)]
    pub struct Displacement(pub f64, pub f64, pub f64);
    #[derive(Clone, Copy)]
    pub struct Velocity(pub f64, pub f64, pub f64);
    #[derive(Clone, Copy)]
    pub struct Acceleration(pub f64, pub f64, pub f64);

    impl Displacement {
        pub fn distance_sq(&self, other: &Displacement) -> f64 {
            square(self.0 - other.0) + square(self.1 - other.1) + square(self.2 - other.2)
        }

        pub fn unit_vector(&self) -> Vector {
            let len = (square(self.0) + square(self.1) + square(self.2)).sqrt();
            Vector(self.0 / len, self.1 / len, self.2 / len)
        }

        pub fn mag(&self) -> f64 {
            (square(self.0) + square(self.1) + square(self.2)).sqrt()
        }

        pub fn dot(&self, other: &Vector) -> f64 {
            self.0 * other.0 + self.1 * other.1 + self.2 + other.2
        }

        pub fn dotD(&self, other: &Displacement) -> f64 {
            self.0 * other.0 + self.1 * other.1 + self.2 + other.2
        }

        pub fn dotV(&self, other: &Velocity) -> f64 {
            self.0 * other.0 + self.1 * other.1 + self.2 + other.2
        }

        pub fn toVector(&self) -> Vector {
            Vector(self.0, self.1, self.2)
        }

        // fn copy(&self) -> Displacement {
        //     Displacement(self.0, self.1, self.2)
        // }
    }

    impl Add for Displacement {
        type Output = Self;
        fn add(self, other: Self) -> Self {
            Self(self.0 + other.0, self.1 + other.1, self.2 + other.2)
        }
    }

    impl Sub for Displacement {
        type Output = Self;
        fn sub(self, other: Self) -> Self {
            Self(self.0 - other.0, self.1 - other.1, self.2 - other.2)
        }
    }

    impl Div<f64> for Displacement {
        type Output = Self;

        fn div(self, f: f64) -> Self {
            Self(self.0/f, self.1/f, self.2/f)
        }
    }

    impl Mul<f64> for Displacement {
        type Output = Self;

        fn mul(self, f: f64) -> Self {
            Self(self.0*f, self.1*f, self.2*f)
        }
    }

    use rustc_serialize::json::{ToJson, Json};
    use std::collections::BTreeMap;

    impl ToJson for Displacement {
        fn to_json(&self) -> Json {
            let mut d = BTreeMap::new();
            // All standard types implement `to_json()`, so use it
            d.insert("x".to_string(), self.0.to_json());
            d.insert("y".to_string(), self.1.to_json());
            d.insert("z".to_string(), self.2.to_json());
            Json::Object(d)
        }
    }

    impl Velocity {
        pub fn multiply_integrate(&self, t: f64) -> Displacement {
            Displacement(self.0 * t, self.1 * t, self.2 * t)
        }

        pub fn mag(&self) -> f64 {
            (square(self.0) + square(self.1) + square(self.2)).sqrt()
        }

        pub fn mag_sq(&self) -> f64 {
            square(self.0) + square(self.1) + square(self.2)
        }

        pub fn unit_vector(&self) -> Vector {
            let len = self.mag();
            Vector(self.0 / len, self.1 / len, self.2 / len)
        }

        pub fn copy(&self) -> Velocity {
            Velocity(self.0, self.1, self.2)
        }

        pub fn toVector(&self) -> Vector {
            Vector(self.0, self.1, self.2)
        }
    }

    impl ToJson for Velocity {
        fn to_json(&self) -> Json {
            let mut d = BTreeMap::new();
            // All standard types implement `to_json()`, so use it
            d.insert("x".to_string(), self.0.to_json());
            d.insert("y".to_string(), self.1.to_json());
            d.insert("z".to_string(), self.2.to_json());
            Json::Object(d)
        }
    }

    impl Add for Velocity {
        type Output = Self;
        fn add(self, other: Self) -> Self {
            Self(self.0 + other.0, self.1 + other.1, self.2 + other.2)
        }
    }

    impl Sub for Velocity {
        type Output = Self;
        fn sub(self, other: Self) -> Self {
            Self(self.0 - other.0, self.1 - other.1, self.2 - other.2)
        }
    }

    impl Div<f64> for Velocity {
        type Output = Self;

        fn div(self, f: f64) -> Self {
            Self(self.0/f, self.1/f, self.2/f)
        }
    }

    impl Mul<f64> for Velocity {
        type Output = Self;

        fn mul(self, f: f64) -> Self {
            Self(self.0*f, self.1*f, self.2*f)
        }
    }

    impl Acceleration {
        pub fn multiply_integrate(&self, t: f64) -> Velocity {
            Velocity(self.0 * t, self.1 * t, self.2 * t)
        }

        pub fn unit_vector(&self) -> Vector {
            let len = (square(self.0) + square(self.1) + square(self.2)).sqrt();
            Vector(self.0 / len, self.1 / len, self.2 / len)
        }

        pub fn mag(&self) -> f64 {
            (square(self.0) + square(self.1) + square(self.2)).sqrt()
        }

    }

    impl Add for Acceleration {
        type Output = Self;
        fn add(self, other: Self) -> Self {
            Self(self.0 + other.0, self.1 + other.1, self.2 + other.2)
        }
    }

    impl Sub for Acceleration {
        type Output = Self;
        fn sub(self, other: Self) -> Self {
            Self(self.0 - other.0, self.1 - other.1, self.2 - other.2)
        }
    }

    impl Div<f64> for Acceleration {
        type Output = Self;

        fn div(self, f: f64) -> Self {
            Self(self.0/f, self.1/f, self.2/f)
        }
    }

    impl Mul<f64> for Acceleration {
        type Output = Self;

        fn mul(self, f: f64) -> Self {
            Self(self.0*f, self.1*f, self.2*f)
        }
    }
}

pub mod advanced {
    use super::basic::*;
    use std::ops::{AddAssign, Div, Mul, Add};

    #[derive(Clone, Copy)]
    pub struct State(pub Displacement, pub Velocity);
    
    pub struct Derivative(pub Velocity, pub Acceleration);
    
    impl Derivative {
        pub fn multiply_integrate(&self, t: f64) -> State {
            State(self.0.multiply_integrate(t), self.1.multiply_integrate(t))
        }
    }
    
    #[derive(Clone, Copy)]
    pub struct Particle {
        pub state: State,
        pub mass: f64,
        pub size: f64
    }
    
    impl AddAssign for State {        
        fn add_assign(&mut self, other: Self) {
            *self = Self(self.0 + other.0, self.1 + other.1);
        }
    }
    
    impl Add for State {
        type Output = Self;
    
        fn add(self, other: State) -> Self::Output {
            State(self.0 + other.0, self.1 + other.1)
        }
    }
    
    impl Div<f64> for State {
        type Output = Self;
    
        fn div(self, other: f64) -> Self::Output {
            State(self.0 / other, self.1 / other)
        }
    }
    
    impl Mul<f64> for State {
        type Output = Self;
    
        fn mul(self, other: f64) -> Self::Output {
            State(self.0 * other, self.1 * other)
        }
    }
    
    impl Particle {
        pub fn print_out(&self) {
            let Displacement(x, y, z) = self.state.0;
            let Velocity(vx, vy, vz) = self.state.1;
            println!("x = <{}, {}, {}> v = <{}, {}, {}>", x, y, z, vx, vy, vz);
        }

        pub fn x(&self) -> f64 { self.state.0.0 }
        pub fn y(&self) -> f64 { self.state.0.1 }
        pub fn z(&self) -> f64 { self.state.0.2 }
        pub fn vx(&self) -> f64 { self.state.1.0 }
        pub fn vy(&self) -> f64 { self.state.1.1 }
        pub fn vz(&self) -> f64 { self.state.1.2 }
    }
    
    pub fn build_particle(x: f64, y: f64, z: f64, vx: f64, vy: f64, vz: f64, mass: f64, size: f64) -> Particle {
        return Particle {
            state: State(
                Displacement(x, y, z),
                Velocity(vx, vy, vz)
            ),
            mass: mass,
            size: size
        }
    }

    use rustc_serialize::json::{ToJson, Json};
    use std::collections::BTreeMap;

    impl ToJson for Particle {
        fn to_json(&self) -> Json {
            let mut d = BTreeMap::new();
            // All standard types implement `to_json()`, so use it
            d.insert("displacement".to_string(), self.state.0.to_json());
            d.insert("velocity".to_string(), self.state.1.to_json());
            Json::Object(d)
        }
    }
}
