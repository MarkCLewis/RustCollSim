

pub mod data {

    pub const PI: f64 = 3.141592653589793;
    pub const K: f64 = 1.0;

    pub mod basic {
        fn square(x: f64) -> f64 { x * x }

        // x, y, z for all
        pub struct Displacement(pub f64, pub f64, pub f64);
        pub struct Velocity(pub f64, pub f64, pub f64);
        pub struct Acceleration(pub f64, pub f64, pub f64);

        impl Displacement {
            pub fn addition(&self, other: &Displacement) -> Displacement {
                Displacement(self.0 + other.0, self.1 + other.1, self.2 + other.2)
            }
    
            pub fn subtraction(&self, other: &Displacement) -> Displacement {
                Displacement(self.0 - other.0, self.1 - other.1, self.2 - other.2)
            }
    
            pub fn distance_sq(&self, other: &Displacement) -> f64 {
                square(self.0 - other.0) + square(self.1 - other.1) + square(self.2 - other.2)
            }
    
            pub fn multiply(&self, n: f64) -> Displacement {
                Displacement(self.0 * n, self.1 * n, self.2 * n)
            }
    
            // fn copy(&self) -> Displacement {
            //     Displacement(self.0, self.1, self.2)
            // }
        }
    
        impl Velocity {
            pub fn multiply_integrate(&self, t: f64) -> Displacement {
                Displacement(self.0 * t, self.1 * t, self.2 * t)
            }
    
            pub fn addition(&self, other: &Velocity) -> Velocity {
                Velocity(self.0 + other.0, self.1 + other.1, self.2 + other.2)
            }
    
            pub fn copy(&self) -> Velocity {
                Velocity(self.0, self.1, self.2)
            }
        }
    
        impl Acceleration {
            pub fn multiply_integrate(&self, t: f64) -> Velocity {
                Velocity(self.0 * t, self.1 * t, self.2 * t)
            }
    
            pub fn addition(&self, other: &Acceleration) -> Acceleration {
                Acceleration(self.0 + other.0, self.1 + other.1, self.2 + other.2)
            }
    
            pub fn multiply(&self, n: f64) -> Acceleration {
                Acceleration(self.0 * n, self.1 * n, self.2 * n)
            }
    
        }
    
    
    }

    pub mod systems {
        use super::basic::*;
        use super::K;

        struct Derivative(pub Velocity, pub Acceleration);

        pub struct Particle {
            pub location: Displacement,
            velocity: Velocity,
            mass: f64,
            size: f64
        }
    
        impl Particle {
            pub fn print_out(&self) {
                let Displacement(x, y, z) = self.location;
                let Velocity(vx, vy, vz) = self.velocity;
                println!("Position = <{}, {}, {}> Velocity = <{}, {}, {}>", x, y, z, vx, vy, vz);
            }
        }

        pub fn build_particle(x: f64, y: f64, z: f64, vx: f64, vy: f64, vz: f64, mass: f64, size: f64) -> Particle {
            return Particle {
                location: Displacement(x, y, z),
                velocity: Velocity(vx, vy, vz),
                mass: mass,
                size: size
            }
        }

        pub struct System(pub Vec<Particle>);

        impl System {

            fn derivative(&self) -> Vec<Derivative> {
                let sys = &self.0;

                let mut deriv: Vec<Derivative> = Vec::new();
            
                for i in 0..(sys.len()) {
            
                    let this = &sys[i];
            
                    let mut acc_vector = Acceleration(0.,0.,0.);
            
                    for j in 0..(sys.len()) {
                        if i != j {
                            let other = &sys[j];
            
                            let d_sq: f64 = this.location.distance_sq(&other.location);
                            let d: f64 = d_sq.sqrt();
            
                            // a = g * m / r^2
                            // g = 1
                            // m = in solar masses
            
                            let acc_scalar: f64 = other.mass / d_sq; // magnitude of a
            
                            let c: Displacement = other.location.subtraction(&this.location).multiply(1.0/d);
                            // now c is a vector Displacementing to b from this
                            // c is displacement unit vector
            
                            let acc = Acceleration(c.0, c.1, c.2); // make a copy
            
                            // c is now a unit vector Displacementing to object j
            
                            let acc = acc.multiply(acc_scalar);
            
                            acc_vector = acc_vector.addition(&acc);
            
                            // i and j forces get calculated twice
            
                            // deal with collisions
                            let delta_x: f64 = d - (this.size + other.size);
                            if delta_x < 0.0 {
                                // intersection!
            
                                // in dir -c (away from other object)
                                let a: f64 = -K * delta_x / this.mass;
            
                                let c = c.multiply(a);
            
                                acc_vector = acc_vector.addition(&Acceleration(c.0, c.1, c.2));
                            }
                        }
                    }
            
            
                    deriv.push(Derivative(this.velocity.copy(), acc_vector));
                } 
            
                return deriv;
            }

            pub fn push(&mut self, p: Particle) {
                self.0.push(p);
            }

            pub fn add_body(&mut self, x: f64, y: f64, z: f64, vx: f64, vy: f64, vz: f64, mass: f64, size: f64) {
                self.0.push(build_particle(x, y, z, vx, vy, vz, mass, size));
            }

            pub fn len(&self) -> usize {
                return self.0.len();
            }

            pub fn kick_step(&mut self, h: f64) {
            
                let deriv: Vec<Derivative> = self.derivative();

                let sys = &mut self.0;

                if sys.len() != deriv.len() {
                    panic!("length mismatch!");
                }
                
                for i in 0..(deriv.len()) {
                    let Derivative(vel, acc) = &deriv[i];
                    let s = &mut sys[i];

                    // kick
                    let vel = vel.addition(&acc.multiply_integrate(h));

                    // step
                    s.velocity = s.velocity.addition(&acc.multiply_integrate(h));
                    s.location = s.location.addition(&vel.multiply_integrate(h));
                }

            }

            // pub fn rk4(&mut self, h: f64) {
            //     /*
            //      * Runge-Kutta
            //      * 
            //      * y' = F(x, y)
            //      * 
            //      * y_n+1 = y_n + 1/6 * (k1 + 2k2 + 2k3 + k4) + O(h^5)
            //      * 
            //      * k1 = hF(xn, yn)
            //      * k2 = hF(xn + h/2, yn + k1/2)
            //      * k3 = hF(xn + h/2, yn + k2/2)
            //      * k4 = hF(xn + h, yn + k3)
            //      * 
            //      * y is position & velocity
            //      * y' is velocity & acceleration
            //      * 
            //      * idk what to do with x for now
            //      */
                
            //     //let mut sysOld = System(self.0.clone());
            //     Vector oldState(data);
            
            //     // k1 = hF(xn, yn)
            //     Vector k1;
            //     derivative(k1);
            //     k1 *= h;
            
            //     // k2 = hF(xn + h/2, yn + k1/2)
            //     (*this) += k1 / 2;
            
            //     Vector k2;
            //     derivative(k2);
            //     k2 *= h;
            
            //     data = oldState;
            //     // k3 = hF(xn + h/2, yn + k2/2)
                
            //     (*this) += k2 / 2;
            
            //     Vector k3;
            //     derivative(k3);
            //     k3 *= h;
            
            //     data = oldState;
            //     // k4 = hF(xn + h, yn + k3)
            
            //     (*this) += k3;
            
            //     Vector k4;
            //     derivative(k4);
            //     k4 *= h;
            
            //     data = oldState;
            //     // 1/6 * (k1 + 2k2 + 2k3 + k4)
            
            //     Vector K(data.size());
            
            //     K += k1;
            
            //     k2 *= 2;
            //     K += k2;
            
            //     k3 *= 2;
            //     K += k3;
            
            //     K += k4;
            
            //     K *= 1.0/6;
            
            //     (*this) += K;
            
            // }

            pub fn print_out(&self) {
                let sys = &self.0;
                
                println!("[");
            
                for p in sys.iter() {
                    print!("  ");
                    p.print_out();
                }
            
                println!("]");
            }
        }

        

        pub fn build_system() -> System {
            return System(Vec::new());
        }
    }
}