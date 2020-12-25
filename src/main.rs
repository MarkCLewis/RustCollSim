
const RHO: f64 = 1.0;

struct Vec3 { x: f64, y: f64, z: f64 }

struct Particle { p: Vec3, v: Vec3, r: f64 }

impl Particle {
    fn m(&self) -> f64 {
        return RHO * self.r * self.r * self.r;
    }
}


fn main() {
    println!("Hello, collisional simulations!");

    let mut bodies = Vec::new();
    bodies.push(Particle { p: Vec3 { x: 0.0, y: 0.0, z: 0.0 }, 
                           v: Vec3 { x: 0.0, y: 0.0, z: 0.0 }, r: 1.0 });
    bodies.push(Particle { p: Vec3 { x: 1.0, y: 0.0, z: 0.0 }, 
                           v: Vec3 { x: 0.0, y: 1.0, z: 0.0 }, r: 1e-4 });
    let dt = 1e-3 * 2.0 * std::f64::consts::PI;
    let mut acc = Vec::new();
    for _ in 0..bodies.len() { 
        acc.push(Vec3 { x: 0.0, y: 0.0, z: 0.0})
    };
    for step in 1..1000001 {
        for i in 0..bodies.len()-1 {
            for j in i+1..bodies.len() {
                acc = calc_accel(i, j, &bodies[i], &bodies[j], acc);
            }
        }
        for i in 0..bodies.len() { 
            bodies[i].v.x += dt * acc[i].x;
            bodies[i].v.y += dt * acc[i].y;
            bodies[i].v.z += dt * acc[i].z;
            bodies[i].p.x += dt * bodies[i].v.x;
            bodies[i].p.y += dt * bodies[i].v.y;
            bodies[i].p.z += dt * bodies[i].v.z;
            acc[i] = Vec3 { x: 0.0, y: 0.0, z: 0.0} 
        };
        if step % 10000 == 0 {
            println!("{} {} {} {} {}", step, bodies[1].p.x, bodies[1].p.y, bodies[1].v.x, bodies[1].v.y);
        }
    }
}

fn calc_accel(i: usize, j: usize, pi: &Particle, pj: &Particle, mut acc: Vec<Vec3>) -> Vec<Vec3> {
    let dx = pi.p.x - pj.p.x;
    let dy = pi.p.y - pj.p.y;
    let dz = pi.p.z - pj.p.z;
    let dist = f64::sqrt(dx*dx + dy*dy + dz*dz);
    let magi = -pj.m() / (dist*dist*dist);
    acc[i].x += dx * magi;
    acc[i].y += dy * magi;
    acc[i].z += dz * magi;
    let magj = pi.m() / (dist*dist*dist);
    acc[j].x += dx * magj;
    acc[j].y += dy * magj;
    acc[j].z += dz * magj;

    return acc;
}
