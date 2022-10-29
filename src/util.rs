use std::cmp::Ordering;

pub fn borrow_two_elements_with_unsafe<'a, T>(
    particles: &'a mut Vec<T>,
    p1: usize,
    p2: usize,
) -> (&'a mut T, &'a mut T) {
    // the raw pointer way of doing it
    assert_ne!(particles.len(), 0); // if Vec is zero-size, then pointer may be dangling
    let ptr = particles.as_mut_ptr();

    unsafe {
        let p1_ptr = ptr.add(p1);
        let p2_ptr = ptr.add(p2);

        (p1_ptr.as_mut().unwrap(), p2_ptr.as_mut().unwrap())
    }
}

pub fn borrow_two_elements<'a, T>(
    particles: &'a mut Vec<T>,
    p1: usize,
    p2: usize,
) -> (&'a mut T, &'a mut T) {
    // there are various ways of doing this, all a bit messy and all don't work well (cries)

    match p1.cmp(&p2) {
        Ordering::Equal => {
            panic!("Cannot make 2 mutable references to same vector element");
        }
        Ordering::Greater => {
            // p1 > p2
            let (first, second) = particles.split_at_mut(p1);
            // p2 is in first, p1 is second[0]
            (&mut first[p2], &mut second[0])
        }
        Ordering::Less => {
            // p1 < p2
            let (first, second) = particles.split_at_mut(p2);
            // p1 is in first, p2 is second[0]
            (&mut first[p1], &mut second[0])
        }
    }
}

pub struct Acceleration(pub [f64; 3]);

use std::ops::{Add, AddAssign};

impl AddAssign for Acceleration {
    fn add_assign(&mut self, other: Self) {
        self.0[0] += other.0[0];
        self.0[1] += other.0[1];
        self.0[2] += other.0[2];
    }
}

impl Add for Acceleration {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Acceleration([
            self.0[0] + other.0[0],
            self.0[1] + other.0[1],
            self.0[2] + other.0[2],
        ])
    }
}
