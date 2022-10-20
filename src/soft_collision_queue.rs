/**
 * This file contains code elements used for the higher time resolution integration of close interactions.
 */

#[derive(Clone, Copy)]
struct ForceEvent {
  time: f64,
  p1: usize,
  p2: usize,
  max_vel: f64,
}