use core::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, Neg, Sub, SubAssign};

#[inline(always)]
fn square(x: f64) -> f64 {
    x * x
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    pub fn iter() -> impl Iterator<Item = Axis> {
        [Axis::X, Axis::Y, Axis::Z].iter().copied()
    }
}

// x, y, z for all
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector(pub [f64; 3]);

impl fmt::Display for Vector {
    // This trait requires `fmt` with this exact signature.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "<{:.3e}, {:.3e}, {:.3e}>",
            self.0[0], self.0[1], self.0[2]
        )
    }
}

impl Vector {
    pub const ZERO: Self = Self([0., 0., 0.]);
    pub const X_HAT: Self = Self([1., 0., 0.]);
    pub const Y_HAT: Self = Self([0., 1., 0.]);
    pub const Z_HAT: Self = Self([0., 0., 1.]);

    #[inline(always)]
    pub fn dot(&self, other: &Vector) -> f64 {
        self.0[0] * other.0[0] + self.0[1] * other.0[1] + self.0[2] * other.0[2]
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

    #[inline(always)]
    pub fn x(&self) -> f64 {
        self.0[0]
    }

    #[inline(always)]
    pub fn y(&self) -> f64 {
        self.0[1]
    }

    #[inline(always)]
    pub fn z(&self) -> f64 {
        self.0[2]
    }

    pub fn min_of_every_axis(&self, other: &Self) -> Self {
        Self([
            self.0[0].min(other.0[0]),
            self.0[1].min(other.0[1]),
            self.0[2].min(other.0[2]),
        ])
    }

    pub fn max_of_every_axis(&self, other: &Self) -> Self {
        Self([
            self.0[0].max(other.0[0]),
            self.0[1].max(other.0[1]),
            self.0[2].max(other.0[2]),
        ])
    }
}

// useful for iterating over dimensions of a vector
impl Index<Axis> for Vector {
    type Output = f64;

    fn index(&self, index: Axis) -> &Self::Output {
        match index {
            Axis::X => &self.0[0],
            Axis::Y => &self.0[1],
            Axis::Z => &self.0[2],
        }
    }
}

impl IndexMut<Axis> for Vector {
    fn index_mut(&mut self, index: Axis) -> &mut Self::Output {
        match index {
            Axis::X => &mut self.0[0],
            Axis::Y => &mut self.0[1],
            Axis::Z => &mut self.0[2],
        }
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

impl DivAssign<f64> for Vector {
    fn div_assign(&mut self, f: f64) {
        *self = Self([self.0[0] / f, self.0[1] / f, self.0[2] / f]);
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

impl Neg for Vector {
    type Output = Vector;

    fn neg(self) -> Self::Output {
        self * -1.
    }
}
