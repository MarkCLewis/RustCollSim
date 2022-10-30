use std::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};

#[inline(always)]
fn square(x: f64) -> f64 {
    x * x
}

// x, y, z for all
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector(pub [f64; 3]);

impl Vector {
    #[inline(always)]
    pub fn dot(&self, other: &Vector) -> f64 {
        self.0[0] * other.0[0] + self.0[1] * other.0[1] + self.0[2] + other.0[2]
    }

    pub fn print(&self) {
        println!("<{:e}, {:e}, {:e}>", self.0[0], self.0[1], self.0[2]);
    }

    pub fn to_str(&self) -> String {
        format!("<{:e}, {:e}, {:e}>", self.0[0], self.0[1], self.0[2])
    }

    #[inline(always)]
    pub fn is_finite(&self) -> bool {
        self.0[0].is_finite() && self.0[1].is_finite() && self.0[2].is_finite()
    }

    #[inline(always)]
    pub fn mag(&self) -> f64 {
        (square(self.0[0]) + square(self.0[1]) + square(self.0[2])).sqrt()
    }

    #[inline(always)]
    pub fn mag_sq(&self) -> f64 {
        square(self.0[0]) + square(self.0[1]) + square(self.0[2])
    }

    #[inline(always)]
    pub fn unit_vector(&self) -> Vector {
        self / self.mag()
    }
}

impl Add for Vector {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self([
            self.0[0] + other.0[0],
            self.0[1] + other.0[1],
            self.0[2] + other.0[2],
        ])
    }
}

impl AddAssign for Vector {
    fn add_assign(&mut self, other: Self) {
        *self = Self([
            self.0[0] + other.0[0],
            self.0[1] + other.0[1],
            self.0[2] + other.0[2],
        ]);
    }
}

impl Sub for Vector {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self([
            self.0[0] - other.0[0],
            self.0[1] - other.0[1],
            self.0[2] - other.0[2],
        ])
    }
}

impl SubAssign for Vector {
    fn sub_assign(&mut self, other: Self) {
        *self = Self([
            self.0[0] - other.0[0],
            self.0[1] - other.0[1],
            self.0[2] - other.0[2],
        ]);
    }
}

impl Div<f64> for Vector {
    type Output = Self;

    fn div(self, f: f64) -> Self::Output {
        Self([self.0[0] / f, self.0[1] / f, self.0[2] / f])
    }
}

impl Div<f64> for &Vector {
    type Output = Vector;

    fn div(self, f: f64) -> Self::Output {
        Vector([self.0[0] / f, self.0[1] / f, self.0[2] / f])
    }
}

impl Mul<f64> for Vector {
    type Output = Self;

    fn mul(self, f: f64) -> Self::Output {
        Self([self.0[0] * f, self.0[1] * f, self.0[2] * f])
    }
}

impl Mul for Vector {
    type Output = f64;

    fn mul(self, f: Self) -> Self::Output {
        self.dot(&f)
    }
}
