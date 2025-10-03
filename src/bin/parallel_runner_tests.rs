// 
#![feature(test)]

use std::time::Instant;
use rust_coll_sim::util::parallel_subset_process::{parallel_subset_process_recur, parallel_subset_process_recur_mut, vector_of_independent_slices};
use rayon::{prelude::*};

#[derive(Clone, Copy)]
struct TestStruct {
  pub x: [f64; 3],
  pub v: [f64; 3],
}

impl TestStruct {
  pub fn new() -> Self {
    Self {
      x: [0.0; 3],
      v: [0.0; 3],
    }
  }

  pub fn random() -> Self {
    Self {
      x: [fastrand::f64(); 3],
      v: [fastrand::f64(); 3],
    }
  }
}

fn main() {
    let data = random_vector(100000000);
    let elements = (0..(data.len()/5)).map(|i| i*5).collect::<Vec<usize>>();
    let now = Instant::now();
    print!("Start first test\n");
    let dots = parallel_subset_process_recur(&data, &elements, &work_with_single_struct);
    println!("dots.len() = {}", dots.len());
    println!("{}", dots[0]);
    println!("{}", dots[dots.len()-1]);
    let elapsed = now.elapsed();
    println!("parallel_subset_process_recur took: {:.2?}", elapsed);

    let now = Instant::now();
    print!("Start second test\n");
    let slices = vector_of_independent_slices(&data, &elements);
    let mut dots = Vec::new();
    dots.resize(elements.len(), 0.0);
    slices.par_iter().map(|s| { work_with_single_struct(&s[0]) }).collect_into_vec(&mut dots);
    println!("dots.len() = {}", dots.len());
    println!("{}", dots[0]);
    println!("{}", dots[dots.len()-1]);
    let elapsed = now.elapsed();
    println!("rayon par_iter took: {:.2?}", elapsed);

    let now = Instant::now();
    print!("Start third test\n");
    let mut dots = Vec::new();
    dots.resize(data.len(), 0.0);
    parallel_subset_process_recur_mut(&data, &mut dots, &elements, &|&p, d| { 
        work_with_single_struct_mut(&p, d)
    });
    println!("dots.len() = {}", dots.len());
    println!("{}", dots[0]);
    println!("{}", dots[dots.len()-5]);
    let elapsed = now.elapsed();
    println!("parallel_subset_process_recur_mut took: {:.2?}", elapsed);
}

fn random_vector(n: usize) -> Vec<TestStruct> {
    let mut ret = Vec::new();
    ret.resize(n, TestStruct::new());
for i in 0..n {
        ret[i] = TestStruct::random();
    }
    ret
}

fn work_with_single_struct(p: &TestStruct) -> f64 {
    let mut sum = 0.0;
    for i in 0..10000 {
        sum += i as f64 * (p.x[0]*p.v[0] + p.x[1]*p.v[1] + p.x[2]*p.v[2]);
    }
    sum
}

fn work_with_single_struct_mut(p: &TestStruct, result: &mut f64) {
    *result = 0.0;
    for i in 0..10000 {
        *result += i as f64 * (p.x[0]*p.v[0] + p.x[1]*p.v[1] + p.x[2]*p.v[2]);
    }
}


extern crate test;

#[cfg(test)]
mod tests {
    use super::*;
    use test::Bencher;
    use rust_coll_sim::util::parallel_subset_process::{parallel_subset_process_recur, parallel_subset_process_recur_mut, vector_of_independent_slices};
    use rayon::{prelude::*};
    use pretty_assertions::{assert_eq};

    #[test]
    fn it_works1() {
      let data = random_vector(100);
      let elements = (0..20).map(|i| i*5).collect::<Vec<usize>>();
      let dots = parallel_subset_process_recur(&data, &elements, &|p1| { 
        p1.x[0]*p1.v[0] + p1.x[1]*p1.v[1] + p1.x[2]*p1.v[2]
      });
      assert_eq!(dots.len(), elements.len());
      for i in 0..dots.len() {
        let d = &data[elements[i]];
        let dot = d.x[0]*d.v[0] + d.x[1]*d.v[1] + d.x[2]*d.v[2];
        assert_eq!(dots[i], dot);
      }
    }


    #[bench]
    fn parallel_dot1(b: &mut Bencher) {
        b.iter(|| {
            let data = random_vector(1000000);
            let elements = (0..(data.len()/5)).map(|i| i*5).collect::<Vec<usize>>();
            let dots = parallel_subset_process_recur(&data, &elements, &|p| { 
                p.x[0]*p.v[0] + p.x[1]*p.v[1] + p.x[2]*p.v[2]
            });
            assert_eq!(dots.len(), elements.len());
        });
    }


    #[test]
    fn it_works2() {
        let data = random_vector(100);
        let elements = (0..20).map(|i| i*5).collect::<Vec<usize>>();
        let slices = vector_of_independent_slices(&data, &elements);
        let mut dots = Vec::new();
        //dots.resize(elements.len(), 0.0);
        slices.into_par_iter().map(|s| {
                s[0].x[0]*s[0].v[0] + s[0].x[1]*s[0].v[1] + s[0].x[2]*s[0].v[2]
        }).collect_into_vec(&mut dots);
        print!("{}", dots.len());
        assert_eq!(dots.len(), elements.len());
        for i in 0..dots.len() {
            let d = &data[elements[i]];
            let dot = d.x[0]*d.v[0] + d.x[1]*d.v[1] + d.x[2]*d.v[2];
            assert_eq!(dots[i], dot);
        }
    }

    #[bench]
    fn parallel_dot2(b: &mut Bencher) {
        b.iter(|| {
            let data = random_vector(1000000);
            let elements = (0..(data.len()/5)).map(|i| i*5).collect::<Vec<usize>>();
            let slices = vector_of_independent_slices(&data, &elements);
            let mut dots = Vec::new();
            dots.resize(elements.len(), 0.0);
            slices.par_iter().map(|s| {
                 s[0].x[0]*s[0].v[0] + s[0].x[1]*s[0].v[1] + s[0].x[2]*s[0].v[2]
            }).collect_into_vec(&mut dots);
            assert_eq!(dots.len(), elements.len());
        });
    }

    #[test]
    fn it_works3() {
        let data = random_vector(100);
        let elements = (0..20).map(|i| i*5).collect::<Vec<usize>>();
        let mut dots = Vec::new();
        dots.resize(data.len(), 0.0);
        parallel_subset_process_recur_mut(&data, &mut dots, &elements, &|p1, d| { 
            *d = p1.x[0]*p1.v[0] + p1.x[1]*p1.v[1] + p1.x[2]*p1.v[2]
        });
        assert_eq!(dots.len(), data.len());
        for i in 0..elements.len() {
            let d = &data[elements[i]];
            let dot = d.x[0]*d.v[0] + d.x[1]*d.v[1] + d.x[2]*d.v[2];
            assert_eq!(dots[elements[i]], dot);
        }
    }

    #[bench]
    fn parallel_dot3(b: &mut Bencher) {
        b.iter(|| {
            let data = random_vector(1000000);
            let elements = (0..(data.len()/5)).map(|i| i*5).collect::<Vec<usize>>();
            let mut dots = Vec::new();
            dots.resize(data.len(), 0.0);
            parallel_subset_process_recur_mut(&data, &mut dots, &elements, &|&p, d| { 
                *d = p.x[0]*p.v[0] + p.x[1]*p.v[1] + p.x[2]*p.v[2]
            });
            assert_eq!(dots.len(), data.len());
        });
    }
                
}