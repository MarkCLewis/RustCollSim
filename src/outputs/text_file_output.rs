// Basic output that write all the particle data to a text file.

use std::fs::File;
use std::io::Write;


use crate::design::system::{Output, Particle, Population};

pub struct TextFileOutput {
  interval: i64,
  file: File,
}

impl TextFileOutput {
  pub fn new(interval: i64, file_name: &str) -> TextFileOutput {
    TextFileOutput { interval, file: File::create(file_name).unwrap() }
  }
}

impl Output for TextFileOutput {
    fn output<P: Population>(&mut self, step: i64, pop: &P) {
      if step % self.interval == 0 {
        for (i, Particle { x, v, m, r, time }) in pop.particles().iter().enumerate() {
          writeln!(&mut self.file, "{} {} {} {} {} {} {} {} {} {} {}",step, i, x.x(), x.y(), x.z(), v.x(), v.y(), v.z(), m, r, time);
        }
      }
    }
}