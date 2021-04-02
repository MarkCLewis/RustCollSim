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

    pub fn toStr(&self) -> String {
        format!("<{:e}, {:e}, {:e}>", self.0, self.1, self.2)
    }

    pub fn is_finite(&self) -> bool {
        self.0.is_finite() && self.1.is_finite() && self.2.is_finite()
    }

    pub fn mag(&self) -> f64 {
        (square(self.0) + square(self.1) + square(self.2)).sqrt()
    }

    pub fn magSq(&self) -> f64 {
        square(self.0) + square(self.1) + square(self.2)
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
