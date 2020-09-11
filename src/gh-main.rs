// struct Position {
//     x: f32,
//     y: f32,
//     z: f32
// }

// struct Velocity {
//     x: f32,
//     y: f32,
//     z: f32
// }

struct Vector {
    x: f64,
    y: f64,
    z: f64
}

struct Particle {
    pos: Vector,
    vel: Vector,
    acc: Vector,
    mass: f64
}

fn main() {
    println!("Welcome to the Kick Step Integrator!");
    let px: f64 = 0.0;
    let py: f64 = 0.0;
    let pz: f64 = 0.0;
    let vx: f64 = 0.0;
    let vy: f64 = 0.0;
    let vz: f64 = 0.0;
    let ax: f64 = 0.0;
    let ay: f64 = 0.0;
    let az: f64 = 0.0;

    let earthM: f64 = 1e-10;
    let sunM: f64 = 1;

    let eParticle = Particle {pos: Vector{x: px, y: py, z: pz,}, 
                                vel: Vector{x: vx, y: vy, z: vz }, 
                                avect: Vector{a: ax, b: ay, c: az},
                                mass: earthM };

    let sParticle = Particle {pos: Vector{x: px, y: py, z: pz,}, 
                                vel: Vector{x: vx, y: vy, z: vz }, 
                                avect: Vector{a: ax, b: ay, c: az},
                                mass: sunM };

    // array of particles
    // outer for loop (timestep)
        // doubly nested loop that calculates acceleration btwn two particles
        // loop applies accelerations, then velocities (outside)
}

fn force(mass1: f64, mass2: f64, r: f64) {
    //vector quantities between two bodies
 
    // calculate distance, use that
    (-1 * (m1 * m2)) / (r*r*r);    // multiply by seperation between them (only vector), divide by r^3
}
fn velocity (par: Particle) { // give back particle? or pass in mut
    //v + dt * a;
    particle.vel.x = particle.vel.x + timestep * particle.acc.x;
    particle.vel.y = particle.vel.y + timestep * particle.acc.y;
    particle.vel.z = particle.vel.z + timestep * particle.acc.z;
    // done to x, y, z separately @ v
}

fn updatePosition (par: Particle, timestep: f64) {
    particle.pos.x = particle.pos.x + timestep * particle.vel.x;
    particle.pos.y = particle.pos.y + timestep * particle.vel.y;
    particle.pos.z = particle.pos.z + timestep * particle.vel.z;
    // done to x, y, z separately @ x 
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
