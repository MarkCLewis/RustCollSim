struct Position {
    x: f32,
    y: f32,
    z: f32
}

struct Velocity {
    x: f32,
    y: f32,
    z: f32
}

struct A {
    a: f32,
    b: f32,
    c: f32
}

struct Particle {
    pos: Position,
    vel: Velocity,
    avect: A
}

fn main() {
    println!("Welcome to the Kick Step Integrator!");
    let px: f32 = 0.0;
    let py: f32 = 0.0;
    let pz: f32 = 0.0;
    let vx: f32 = 0.0;
    let vy: f32 = 0.0;
    let vz: f32 = 0.0;
    let a: f32 = 0.0;
    let b: f32 = 0.0;
    let c: f32 = 0.0;

    let particle = Particle {pos: Position{x: px, y: py, z: pz}, 
                             vel: Velocity{x: vx, y: vy, z: vz }, 
                             avect: A{a: a, b: b, c: c} };
}

fn force(g: i32, m1: i32, m2: i32, r: i32) {
    ((-1 * g) * (m1 * m2)) / (r*r);
}
fn velocity(v: i32, dt: i32, a: i32) {
    v + dt * a;
}
fn updatePosition(x: i32, dt: i32, v: i32) {
    x + dt * v; 
}

// Take all particles
// Calculate force on all of them (acceleration)
// v' = v + dt*a
// New vel = old vel + small timestep * a
// dt = 1e-3
// Step updates position using new velocity
// x' = x + dt*v'
// A, v, x are vectors
// All z will be 0
// For every particle, there is a position (x, y, z) and a velocity (x, y, z) and an A (a, b, c)
// Write code for an array of particles
// Gravity calculated between two paired particles
// Two nested loops to run thru every i, j pair
