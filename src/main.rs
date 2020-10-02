#![allow(dead_code)]

mod data;
mod graphics;
mod system;

use data::PI;

extern crate rustc_serialize;
use rustc_serialize::json;

use system::data_storage_help::*;
use rustc_serialize::json::{ToJson}; 


fn test_group_2_kick_step_1() {
    let h: f64 = 1e-3;

    let mut sys = system::build_system(-1., -1.);

    sys.push(data::advanced::build_particle(0.,0.,0.,0.,0.,0., 1.0, 1e-7));
    sys.push(data::advanced::build_particle(1.,0.,0.,0.,1.,0.,1e-10, 1e-7));

    println!("N-body simulation");
    println!("Number of bodies: {}" , sys.len());

    let mut i: f64 = 0.0;
    while i < 2.0*PI {
        sys.kick_step(h);

        i += h;
    }

    println!("Final Result:");
    sys.print_out();
}

fn test_group_3_rk4_1() {
    let h: f64 = 1e-3;

    let mut sys = system::build_system(-1., -1.);

    sys.push(data::advanced::build_particle(0.,0.,0.,0.,0.,0., 1.0, 1e-7));
    sys.push(data::advanced::build_particle(1.,0.,0.,0.,1.,0.,1e-10, 1e-7));

    println!("N-body simulation");
    println!("Number of bodies: {}" , sys.len());

    let mut i: f64 = 0.0;
    while i < 2.0*PI {
        sys.rk4(h);

        i += h;
    }

    println!("Final Result:");
    sys.print_out();
}

fn test_group_5_graphics(orbits: i32) {
    let h: f64 = 1e-2;

    let mut sys = system::build_system(-1., -1.);

    sys.add_body(0., 0., 0., 0., 0., 0., 1., 0.00465479256);
    sys.add_body(1., 0., 0., 0., 1., 0., 1e-8, 4.25879793e-5);

    // std::cerr << "N-body simulation\n";
    // std::cerr << "Number of bodies: " << sys.size() << '\n';

    let mut g = graphics::build_graphics();
    g.init();



    let mut min: i32 = g.get_max_x();
    if g.get_max_y() < min {
        min = g.get_max_y();
    }

    let factor: f64 = min as f64 / 2.5;

    g.set_scale_factor(factor);

    let mut i: f64 = 0.0;
    while i < orbits as f64 * 2.0 * PI {

        for (c, p) in sys.state.iter().enumerate() {
            g.draw_point(p.state.0.0, p.state.0.1, '.' as u64, (c % 4) as i16);
        }

        sys.kick_step(h);

        for (c, p) in sys.state.iter().enumerate() {
            g.draw_point(p.state.0.0, p.state.0.1, 'o' as u64, (c % 4) as i16);

        }
        g.refresh();

        info(i, &mut g, &sys);

        graphics::sleep(10000);

        i += h;
    }

    g.end();
}

fn test_group_6_collision() {
    let h = 1e-2;

    let mut sys = system::build_system(-1., -1.);

    sys.add_body(-1e-1, 0., 0., 0., 0., 0., 1e-2, 4.25879793e-5);
    sys.add_body(1e-1, 0., 0., 0., 0., 0., 1e-2, 4.25879793e-5);

    // std::cerr << "N-body simulation\n";
    // std::cerr << "Number of bodies: " << sys.size() << '\n';

    let mut g = graphics::build_graphics();
    g.init();

    let mut min: i32 = g.get_max_x();
    if g.get_max_y() < min {
        min = g.get_max_y();
    }

    let factor: f64 = min as f64 / (1e-0 * 2.5);

    g.set_scale_factor(factor);

    let mut i: f64 = 0.0;
    while i < 100. * PI * 2. as f64 * 2.0 * PI {
        //clear();

        for (c, p) in sys.state.iter().enumerate() {
            g.draw_point(p.state.0.0, p.state.0.1, '.' as u64, (c % 4) as i16);
        }

        sys.rk4(h);

        for (c, p) in sys.state.iter().enumerate() {
            g.draw_point(p.state.0.0, p.state.0.1, 'o' as u64, (c % 4) as i16);

        }
        g.refresh();

        graphics::sleep(10000);

        i += h;
    }

    g.end();
}

pub fn collision(k: f64, drag: f64) {
    let mut g = graphics::build_graphics();
    g.init();

    let mut min: i32 = g.get_max_x();
    if g.get_max_y() < min {
        min = g.get_max_y();
    }

    let factor: f64 = min as f64 / (1e-7 * 3.);

    g.set_scale_factor(factor);

    // std::cerr << "Survived g.init\n";

    let h: f64 = 1e-6;
    //std::cerr << "> k = " << k << ", drag = " << drag << '\n';
    let mut sys = system::build_system(k, drag);

    // State sys(h, 0, 0, k, drag);

    let r: f64 = 1e-7; // 130000 km

    // 2g/cm^3

    // saturns mass = 5.683e26 kg
    //              = 5.683e29 g
    //
    // radius       = 1.3e5 km
    //              = 1.3e8 m
    //              = 1.3e10 cm
    //
    //                          (1.3e10)^3
    // so conversion factor = --------------
    //                           5.683e29

    let rho: f64 = 7.7; //129000; sat mass/ ring radius^3
    let mass: f64 = 4.0/3.0 * 3.14159 * r * r * r * rho;

    sys.add_body(-2. * r, 0., 0., 2e-6, 0., 0., mass, r);
    sys.add_body(2. * r, 0., 0., -2e-6, 0., 0., mass, r);



    //sys.addBody(0, 35e-4, 0, 5, 0, 0, 1e-2, 4.25879793e-4);
    //sys.addBody(0, -35e-4, 0, -5, 0, 0, 1e-2, 4.25879793e-4);

    //sys.addBody(-30e-5, 35e-5, 0, 0, 5, 0, 1e-2, 4.25879793e-5);
    //sys.addBody(30e-5, -35e-5, 0, 2, 2, 0, 1e-2, 4.25879793e-5);
    //sys.addBody(30e-5, 35e-5, 0, 5, 0, 0, 1e-2, 4.25879793e-5);
    //sys.addBody(-30e-5, -35e-5, 0, -5, 0, 0, 1e-2, 4.25879793e-5);

    //sys.addBody(0, 0, 0, 0, 0, 0, 1, 0.00465479256);
    // sys.addBody(1, 0, 0, 0, 1, 0, 1e-8, 4.25879793e-5);

    //std::cerr << "N-body simulation\n";
    //std::cerr << "Number of bodies: " << sys.size() << '\n';


    //std::cerr << "Survived init" << std::endl;

    let mut i: f64 = 0.0;
    while i < 100. * PI * 2. * 2.0 * PI { 
        //clear();

        for (c, p) in sys.state.iter().enumerate() {
            g.draw_point(p.state.0.0, p.state.0.1, '.' as u64, (c % 4) as i16);
        }

        sys.kick_step(h);

        for (c, p) in sys.state.iter().enumerate() {
            g.draw_point(p.state.0.0, p.state.0.1, 'o' as u64, (c % 4) as i16);
        }

        //double totalE = sys.computeEnergy();

        //g.print(0, 0, )
        
        // mvprintw(0, 0, "Energy = %e", totalE);
        info(i, &mut g, &sys);

        // int c = g.sleepInterruptible(1); // returns 0 on no char typed

        // if (c == 'd') {
        //     g.setScreenRelNoThrow(1);
        // }
        // else if (c == 'a') {
        //     g.setScreenRelNoThrow(-1);
        // }
        // else if (c > 0) {
        //     std::cerr << "c = " << c << '\n';
        //     break; // close program if any key is hit
        // }

        graphics::sleep(100);

        i += h;
    }

    //std::cerr << "Survived run\n";

    g.end();
}

pub fn collision_no_graphics(k: f64, drag: f64, max_store_samples: i32) -> FullSimulationState {
    let h: f64 = 1e-6;
    let mut sys = system::build_system(k, drag);


    let r: f64 = 1e-7; // 130000 km

    // 2g/cm^3

    // saturns mass = 5.683e26 kg
    //              = 5.683e29 g
    //
    // radius       = 1.3e5 km
    //              = 1.3e8 m
    //              = 1.3e10 cm
    //
    //                          (1.3e10)^3
    // so conversion factor = --------------
    //                           5.683e29

    let rho: f64 = 7.7; //129000; sat mass/ ring radius^3
    let mass: f64 = 4.0/3.0 * 3.14159 * r * r * r * rho;

    sys.add_body(-2. * r, 0., 0., 2e-6, 0., 0., mass, r);
    sys.add_body(2. * r, 0., 0., -2e-6, 0., 0., mass, r);

    let mut i: f64 = 0.0;
    let up_to: f64 = 1.5e-1;

    

    let mut output = build_full_simulation_state(&sys);

    let mut counter = 1;

    let mut counter_max = 20;
    if max_store_samples > 0 {
        let time_step_between_samples = up_to / max_store_samples as f64;
        counter_max = (time_step_between_samples / h) as i32;
    }


    while i < up_to { 
        sys.kick_step(h);

        //info(i, &mut g, &sys);
        
        if counter > counter_max {
            output.push_state(&sys, i);
            counter = 0;
        }
        counter += 1;

        i += h;
    }

    if max_store_samples > 0 {
        assert_eq!(true, (output.states.len() as i32 - max_store_samples).abs() <= 1);
    }

    return output;
    

    // to 0.15
}

fn info(i: f64, g: &mut graphics::Graphics, sys: &system::System) {
    let msg = format!("Time = {:.3e}", i);
    g.print(0, 0, &msg[..], graphics::WHITE_ON_BLACK);
    let v0: data::basic::Velocity = sys.get_velocity(0);
    let msg = format!("Velocity 1 = <{:.5e}, {:.5e}, {:.5e}>", v0.0, v0.1, v0.2);
    g.print(0, 1, &msg[..], graphics::WHITE_ON_BLACK);

    let v1: data::basic::Velocity = sys.get_velocity(1);
    let msg2 = format!("Velocity 2 = <{:.5e}, {:.5e}, {:.5e}>", v1.0, v1.1, v1.2);
    //mvprintw(3, 0, "v 1 = <%e, %e, %e>", v0.x, v0.y, v0.z);
    g.print(0, 2, &msg2[..], graphics::WHITE_ON_BLACK);

    let e = sys.energy();
    let msg2 = format!("Energy = {:.5e}", e);
    g.print(0, 3, &msg2[..], graphics::WHITE_ON_BLACK);

    g.print(0, g.get_max_y() - 1, "Press ^C to close", graphics::WHITE_ON_BLACK);

    g.refresh();
}

fn main() {
    eprintln!("Hello, world!");

    //test_group_5_graphics(2);
    //test_group_3_rk4_1();
    //test_group_6_collision();
    let output = collision_no_graphics(1e-8, 1e-15, 10);
    let encoded = json::encode(&output.to_json()).unwrap();
    println!("{}",encoded);
    //test_group_2_kick_step_1();

}
