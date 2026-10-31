use rayon::iter::{IndexedParallelIterator, IntoParallelIterator, IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator};



fn main() {
  let a = vec![1,2,3];
  let b = vec![4,5,6];
  let mut c = Vec::new();
  c.resize(3, 0);

  // (0..c.len()).into_par_iter().for_each(|i| {
  //   c[i] = a[i] + b[i];
  // });

  c.par_iter_mut().enumerate().for_each(|(i, ci)| {
    *ci = a[i] + b[i];
  });

  // let subset = vec![0,2];
  // subset.into_par_iter().for_each(|i| {
  //   c[i] = a[i] + b[i];
  // });
}