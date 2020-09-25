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
    let sunM: f64 = 1.0;

    let eParticle = Particle {pos: Vector{x: px, y: py, z: pz,}, 
                                vel: Vector{x: vx, y: vy, z: vz }, 
                                acc: Vector{x: ax, y: ay, z: az},
                                mass: earthM };

    let sParticle = Particle {pos: Vector{x: px, y: py, z: pz,}, 
                                vel: Vector{x: vx, y: vy, z: vz }, 
                                acc: Vector{x: ax, y: ay, z: az},
                                mass: sunM };

    // array of particles - fxn for reading in from file?

    let particles = [eParticle, sParticle];
    let loops = 31400; // make a user input variable
    let timestep: f64 = 1e-3;
    
    while loops != 0 {
        for i in 0..particles.len() {
            for j in 1..particles.len() {
                let force = force(particles[i], particles[j]);
                let negforce = Vector{x: -1.0 * force.x, y: -1.0 * force.y, z: -1.0 * force.z};

                acceleration(particles[i], force);
                acceleration(particles[j], negforce);
                velocity(particles[i], timestep);
                velocity(particles[j], timestep);

                updatePosition(particles[i], timestep);
                updatePosition(particles[j], timestep);
            }
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

fn printParticle(par: Particle) {
    println!("Particle has position {posx} {posy} {posz}",
    posx=par.pos.x,
    posy=par.pos.y,
    posz=par.pos.z
    );
}

fn acceleration(mut par: Particle, force: Vector) {
    par.acc.x = force.x / par.mass;
    par.acc.y = force.y / par.mass;
    par.acc.z = force.z / par.mass;
}

fn force(par1: Particle, par2: Particle) -> Vector { // make vector values negative for par2
    let dx = par2.pos.x - par1.pos.x;
    let dy = par2.pos.y - par1.pos.y;
    let dz = par2.pos.z - par1.pos.z;

    let r = ((dx*dx) + (dy*dy) + (dz*dz)).sqrt();

    let fx = -dx * (par1.mass * par2.mass) / (r*r*r); 
    let fy = -dy * (par1.mass * par2.mass) / (r*r*r);
    let fz = -dz * (par1.mass * par2.mass) / (r*r*r);
    return Vector{x: fx, y: fy, z: fz};
}

fn velocity (mut par: Particle, timestep: f64) { // give back particle? or pass in mut
    //v + dt * a;
    par.vel.x = par.vel.x + timestep * par.acc.x;
    par.vel.y = par.vel.y + timestep * par.acc.y;
    par.vel.z = par.vel.z + timestep * par.acc.z;
    // done to x, y, z separately @ v
}

fn updatePosition (mut par: Particle, timestep: f64) {
    par.pos.x = par.pos.x + timestep * par.vel.x;
    par.pos.y = par.pos.y + timestep * par.vel.y;
    par.pos.z = par.pos.z + timestep * par.vel.z;
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
