#![allow(non_snake_case)]

// Working Kick-step Integrator
// Gracen Hoyle & Steven Marquez

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

    let eParticle = Particle {pos: Vector{x: 1.0, y: py, z: pz,}, 
                                vel: Vector{x: vx, y: 1.0, z: vz }, 
                                acc: Vector{x: ax, y: ay, z: az},
                                mass: earthM };

    let sParticle = Particle {pos: Vector{x: px, y: py, z: pz,}, 
                                vel: Vector{x: vx, y: vy, z: vz }, 
                                acc: Vector{x: ax, y: ay, z: az},
                                mass: sunM };


    let mut particles = [eParticle, sParticle];
    let mut loops = 3140; 
    let timestep: f64 = 1e-3;
    
    while loops != 0 {
        for i in 0..particles.len() {
            for j in i+1..particles.len() {
                let force = force(&particles[i], &particles[j]);
                println!("Force is: {} {} {}", force.x, force.y, force.z);
                let negforce = Vector{x: -1.0 * force.x, y: -1.0 * force.y, z: -1.0 * force.z};

                acceleration(&mut particles[i], force);
                acceleration(&mut particles[j], negforce);
            }
        }

        for i in 0..particles.len() {
            velocity(&mut particles[i], timestep);
            updatePosition(&mut particles[i], timestep);
            particles[i].acc.x = 0.0;
            particles[i].acc.y = 0.0;
            particles[i].acc.z = 0.0;
            let par = &particles[i];
            println!("Particle[{ind}] has position {posx} {posy} {posz}",
            ind = i,
            posx=par.pos.x,
            posy=par.pos.y,
            posz=par.pos.z
            );
        }
        loops -= 1;
    }

}

fn acceleration(par: &mut Particle, force: Vector) {
    par.acc.x += force.x / par.mass;
    par.acc.y += force.y / par.mass;
    par.acc.z += force.z / par.mass;
}

fn force(par1: &Particle, par2: &Particle) -> Vector { 
    let dx = par2.pos.x - par1.pos.x;
    let dy = par2.pos.y - par1.pos.y;
    let dz = par2.pos.z - par1.pos.z;

    let r = ((dx*dx) + (dy*dy) + (dz*dz)).sqrt();

    let fx = dx * (par1.mass * par2.mass) / (r*r*r); 
    let fy = dy * (par1.mass * par2.mass) / (r*r*r);
    let fz = dz * (par1.mass * par2.mass) / (r*r*r);
    return Vector{x: fx, y: fy, z: fz};
}

fn velocity (par: &mut Particle, timestep: f64) { 
    par.vel.x = par.vel.x + timestep * par.acc.x;
    par.vel.y = par.vel.y + timestep * par.acc.y;
    par.vel.z = par.vel.z + timestep * par.acc.z;

}

fn updatePosition (par: &mut Particle, timestep: f64) {
    par.pos.x = par.pos.x + timestep * par.vel.x;
    par.pos.y = par.pos.y + timestep * par.vel.y;
    par.pos.z = par.pos.z + timestep * par.vel.z;

}