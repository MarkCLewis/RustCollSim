
use crate::data::advanced::*;
use crate::data::basic::*;
use crate::data::{DRAG_DEFAULT, K_DEFAULT};
use itertools::izip;
use std::ops::AddAssign;

pub struct System {
    pub state: Vec<Particle>,
    pub k: f64,
    pub drag: f64
}

impl AddAssign<&Vec<State>> for System {
    fn add_assign(&mut self, state: &Vec<State>) {
        for (p, s) in izip!(self.state.iter_mut(), state.iter()) {
            p.state += *s;
        }
    }
}

impl System {

    pub fn get_velocity(&self, index: usize) -> Velocity {
        self.state[index].state.1
    }

    fn derivative(&self) -> Vec<Derivative> {
        let sys = &self.state;

        let mut deriv: Vec<Derivative> = Vec::new();
    
        for i in 0..(sys.len()) {
    
            let this = &sys[i];
    
            let mut acc_vector = Acceleration(0.,0.,0.);
    
            for j in 0..(sys.len()) {
                if i != j {
                    let other = &sys[j];
    
                    let d_sq: f64 = this.state.0.distance_sq(&other.state.0);
                    let d: f64 = d_sq.sqrt();
    
                    // a = g * m / r^2
                    // g = 1
                    // m = in solar masses
    
                    let acc_scalar: f64 = other.mass / d_sq; // magnitude of a

                    let c = (other.state.0 - this.state.0).unit_vector();
    
                    //let c: Displacement = (other.state.0 - this.state.0) / d;
                    // now c is a vector Displacementing to b from this
                    // c is displacement unit vector
    
                    let acc = Acceleration(c.0, c.1, c.2); // make a copy
    
                    // c is now a unit vector pointing to object j
    
                    let acc = acc * acc_scalar;
    
                    acc_vector = acc_vector + acc;
    
                    // i and j forces get calculated twice
    
                    // deal with collisions
                    let delta_x: f64 = d - (this.size + other.size);
                    if delta_x < 0.0 {
                        // intersection!
                        //let tmp = self.get_velocity(0);
                        //eprintln!("Collision: v=<{:.3e}, {:.3e}, {:.3e}>", tmp.0, tmp.1, tmp.2);


                        let rel: Velocity = other.state.1 -this.state.1;

                        let v_mag: f64 = rel.0 * c.0 + rel.1 * c.1 + rel.2 * c.2;
                        //rel.dot(c);

                        // in dir -c (away from other object)
                        let a: f64 = (-self.k * -delta_x + self.drag * v_mag) / this.mass;
                        
                        let ca = c * a;
                        acc_vector = acc_vector + Acceleration(ca.0, ca.1, ca.2);
    
                        // // in dir -c (away from other object)
                        // let a: f64 = -self.k * delta_x / this.mass;
    
                        // let c = c.multiply(a);
    
                        // acc_vector = acc_vector.addition(&Acceleration(c.0, c.1, c.2));
                    }
                }
            }
    
    
            deriv.push(Derivative(this.state.1.copy(), acc_vector));
        } 
    
        return deriv;
    }

    pub fn push(&mut self, p: Particle) {
        self.state.push(p);
    }

    pub fn add_body(&mut self, x: f64, y: f64, z: f64, vx: f64, vy: f64, vz: f64, mass: f64, size: f64) {
        self.state.push(build_particle(x, y, z, vx, vy, vz, mass, size));
    }

    pub fn len(&self) -> usize {
        return self.state.len();
    }

    pub fn kick_step(&mut self, h: f64) {
    
        let deriv: Vec<Derivative> = self.derivative();

        let sys = &mut self.state;

        if sys.len() != deriv.len() {
            panic!("length mismatch!");
        }
        
        for i in 0..(deriv.len()) {
            let Derivative(vel, acc) = &deriv[i];
            let s = &mut sys[i];

            // kick
            let vel = *vel + acc.multiply_integrate(h);

            // step
            s.state.1 = s.state.1 + acc.multiply_integrate(h);
            s.state.0 = s.state.0 + vel.multiply_integrate(h);
        }

    }

    pub fn rk4(&mut self, h: f64) {
        /*
            * Runge-Kutta
            * 
            * y' = F(x, y)
            * 
            * y_n+1 = y_n + 1/6 * (k1 + 2k2 + 2k3 + k4) + O(h^5)
            * 
            * k1 = hF(xn, yn)
            * k2 = hF(xn + h/2, yn + k1/2)
            * k3 = hF(xn + h/2, yn + k2/2)
            * k4 = hF(xn + h, yn + k3)
            * 
            * y is position & velocity
            * y' is velocity & acceleration
            * 
            * idk what to do with x for now
            */

        /*
            * make a copy of sys
            * final_k = vec<state>
            * k1 = iter derivative() * h
            * loop over k1
            *      add k1 /6 to final_k
            *      add k1 /2 to sys
            * k2 = iter derivative() * h
            * recopy sys
            * loop over k2
            *      add k2 * 2/6 to final_k
            *      add k2 /2 to sys
            * k3 = iter derivative() * h
            * recopy sys
            * loop over k3
            *      add k3 * 2/6 to final_k
            *      add k3 to sys
            * k4 = iter derivative() * h
            * loop over k4
            *      add k4 /6 to final_k
            * 
            * 
            */
        
        let mut sys_copy = build_system2(self.k, self.drag, self.state.clone());
        //System(self.0.clone());
        //Vector oldState(data);
    
        // k1 = hF(xn, yn)
        // Vector k1;
        // derivative(k1);
        let k1: Vec<State> = sys_copy.derivative().into_iter().map(|x| x.multiply_integrate(h)).collect();
        // k1 *= h;
    
        // k2 = hF(xn + h/2, yn + k1/2)
        let k1_tmp = k1.clone();
        let halved: Vec<State> = k1_tmp.into_iter().map(|x| x / 2.0).collect();
        sys_copy += &halved;
        // (*this) += k1 / 2;



        let k2: Vec<State> = self.derivative().into_iter().map(|x| x.multiply_integrate(h)).collect();
        // Vector k2;
        // derivative(k2);
        // k2 *= h;

        sys_copy = build_system2(self.k, self.drag, self.state.clone());
        // data = oldState;
        // // k3 = hF(xn + h/2, yn + k2/2)
        
        // (*this) += k2 / 2;
        let k2_tmp = k2.clone();
        let halved: Vec<State> = k2_tmp.into_iter().map(|x| x / 2.0).collect();
        sys_copy += &halved;
    
        // Vector k3;
        // derivative(k3);
        // k3 *= h;
        let k3: Vec<State> = sys_copy.derivative().into_iter().map(|x| x.multiply_integrate(h)).collect();

        sys_copy = build_system2(self.k, self.drag, self.state.clone());                // data = oldState;
        // // k4 = hF(xn + h, yn + k3)
    
        // (*this) += k3;

        sys_copy += &k3;
    
        // Vector k4;
        // derivative(k4);
        // k4 *= h;
        let k4: Vec<State> = self.derivative().into_iter().map(|x| x.multiply_integrate(h)).collect();
        
        // sys_copy = System(self.0.clone());
        // data = oldState;
        // // 1/6 * (k1 + 2k2 + 2k3 + k4)

        let mut final_k: Vec<State> = Vec::new();
    
        // Vector K(data.size());
    
        // K += k1;
        
    
        // k2 *= 2;
        // K += k2;

    
        // k3 *= 2;
        // K += k3;
    
        // K += k4;

        for i in 0..k1.len() {
            final_k.push( ( k1[i] + k2[i] * 2. + k3[i] * 2. + k4[i] ) / 6.0 );
        }
    
        // K *= 1.0/6;
    
        // (*this) += K;
        *self += &final_k;
    }

    pub fn print_out(&self) {
        let sys = &self.state;
        
        println!("[");
    
        for p in sys.iter() {
            print!("  ");
            p.print_out();
        }
    
        println!("]");
    }
}

pub fn build_system(k: f64, drag: f64) -> System {
    let k = if k < 0.0 { K_DEFAULT } else { k };
    let drag = if drag < 0.0 { DRAG_DEFAULT } else { drag };

    return System {
        state: Vec::new(),
        k: k,
        drag: drag
    };
}

pub fn build_system2(k: f64, drag: f64, state: Vec<Particle>) -> System {
    let k = if k < 0.0 { K_DEFAULT } else { k };
    let drag = if drag < 0.0 { DRAG_DEFAULT } else { drag };

    return System {
        state: state,
        k: k,
        drag: drag
    };
}
