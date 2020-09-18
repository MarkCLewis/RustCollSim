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

    let mut eParticle = Particle {pos: Vector{x: px, y: py, z: pz,}, 
                                vel: Vector{x: vx, y: vy, z: vz }, 
                                avect: Vector{a: ax, b: ay, c: az},
                                mass: earthM };

    let mut sParticle = Particle {pos: Vector{x: px, y: py, z: pz,}, 
                                vel: Vector{x: vx, y: vy, z: vz }, 
                                avect: Vector{a: ax, b: ay, c: az},
                                mass: sunM };

    // array of particles - fxn for reading in from file?

    let particles = [(eParticle, sParticle)];
    let loops = 10; // make a user input variable
    let timestep: f64 = 1e-3;
    
    for i in 0...particles.len() {
        for j in 0...(particles.len() + 1) {
            particles[i]
        }
    }

    // for i in particles.iter() {
    //     velocity(i.0, timestep);
    //     velocity(i.1, timestep);
    //     updatePosition(i.0, timestep);
    //     updatePosition(i.1 , timestep);
    // }
    
    // outer for loop (timestep)
        // doubly nested loop that calculates acceleration btwn two particles
        // loop applies accelerations, then velocities (outside)
}

fn acceleration(par: Particle, force: Vector) {
    let ax = force.x / par.mass;
    let ay = force.y / par.mass;
    let az = force.z / par.mass;
}

fn force(par1: Particle, par2: Particle) -> Vector { // make vector values negative for par2
    let dx = par2.pos.x - par1.pos.x;
    let dy = par2.pos.y - par1.pos.y;
    let dz = par2.pos.z - par1.pos.z;

    let r = sqrt((dx*dx) + (dy*dy) + (dz*dz));

    let fx = -dx * (par1.mass * par2.mass) / (r*r*r); 
    let fy = -dy * (par1.mass * par2.mass) / (r*r*r);
    let fz = -dz * (par1.mass * par2.mass) / (r*r*r);
    Vector(fx, fy, fz);
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
