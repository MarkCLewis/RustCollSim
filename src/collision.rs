#![allow(non_snake_case)]

use crate::system;
use crate::data::PI;
use crate::system::data_storage_help::*;
use crate::graphics;
use itertools::izip;
use rand::Rng;



pub fn collision(k: f64, drag: f64) {
    let r: f64 = 1e-7; // 130000 km
    /*
     * diameter = 2e-7
     * area(sq) = 4e-14 
     * 1000 -> 4e-11
     * 1% space covered, aka * 100
     * sqrt( 4e-9 )
     * 6.324555320336759e-05
     */
    let areaAsSquare = (2.*r)*(2.*r);

    let count = 1000;
    let areaCovered = areaAsSquare * count as f64;

    let fullAreaCovered = areaCovered * 100.;
    let edgeLen = fullAreaCovered.sqrt();

    // 

    let cellSize = edgeLen;
    //10e-7;//3e-4;

    let mut g = graphics::build_graphics();
    g.init();

    let mut min: i32 = g.get_max_x();
    if g.get_max_y() < min {
        min = g.get_max_y();
    }
    min -= 2; // because box

    let factor: f64 = min as f64 / cellSize;

    g.set_scale_factor(factor);

    

    // std::cerr << "Survived g.init\n";

    let h: f64 = 1e-5;//1e-6;
    //std::cerr << "> k = " << k << ", drag = " << drag << '\n';
    let mut sys = system::build_system(k, drag);

    // State sys(h, 0, 0, k, drag);

    

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

    //sys.add_body(-2. * r, 0., 0., 2e-6, 0., 0., mass, r);
    //sys.add_body(2. * r, 0., 0., -2e-6, 0., 0., mass, r);
    //sys.add_body(2e-7, 1e-7, 0., 1e-7, 0., 0., mass, r);
    // sys.add_body(0., 5e-7, 0., 1e-7, 0., 0., mass, r);
    // sys.add_body(0., 1e-7, 0., 1e-7, 0., 0., mass, r);

    let mut rng = rand::thread_rng();
    
    for _ in 0..count {
        sys.add_body(
            rng.gen_range(-cellSize, cellSize), 
            rng.gen_range(-cellSize, cellSize), 
            rng.gen_range(-cellSize, cellSize), 
            0.,//rng.gen_range(-1e-7, 1e-7), 
            0.,//rng.gen_range(-1e-7, 1e-7), 
            0.,//rng.gen_range(-1e-7, 1e-7), 
            mass, r);
    }


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

    let mut counter = 0;
    const COUNTER_MAX: i32 = 600;

    let mut lastPos: Vec<graphics::GraphicsPoint> = Vec::new();

    let mut i: f64 = 0.0;
    while i < 100. * PI * 2. * 2.0 * PI { 
        //clear();

        if counter > COUNTER_MAX {


            for (p, (c, pa)) in izip!(lastPos.iter(), sys.state.iter().enumerate()) {
                let color = (c % 4) as i16;
                g.draw_point(p.x - pa.size, p.y, ' ' as u64, color);
                g.draw_point(p.x + pa.size, p.y, ' ' as u64, color);
                g.draw_point(p.x, p.y - pa.size, ' ' as u64, color);
                g.draw_point(p.x, p.y + pa.size, ' ' as u64, color);
                g.draw_point(p.x, p.y, '.' as u64, (c % 4) as i16);
            }
            lastPos.clear();
        }

        sys.kick_step(h);

        for p in sys.state.iter_mut() {
            system::System::slidingBrickBoundary(p, i, cellSize, cellSize);
        }

        if counter > COUNTER_MAX {
            //g.clear();
            for (c, p) in sys.state.iter().enumerate() {
                let color = (c % 4) as i16;

                g.draw_point(p.state.0.0 - p.size, p.state.0.1, '|' as u64, color);
                g.draw_point(p.state.0.0 + p.size, p.state.0.1, '|' as u64, color);
                g.draw_point(p.state.0.0, p.state.0.1 - p.size, '-' as u64, color);
                g.draw_point(p.state.0.0, p.state.0.1 + p.size, '-' as u64, color);
                g.draw_point(p.state.0.0, p.state.0.1, 'o' as u64, color);

                lastPos.push(graphics::build_graphics_point(p.state.0.0, p.state.0.1));
            }

            //double totalE = sys.computeEnergy();

            //g.print(0, 0, )
            
            // mvprintw(0, 0, "Energy = %e", totalE);
            g.centeredBox();
            g.info(i, &sys);

            counter = 0;

        }
        counter += 1;

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

    

    let mut output = build_full_simulation_state(&sys, h, "kick_step".to_string());

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

pub fn collision_drag_analysis(k: f64, drags: Vec<f64>) {

    for &drag in drags.iter() {
        let out = collision_no_graphics(k, drag, 10);

        let firstE = out.states[0].total_e;
        let lastE = out.states[out.states.len()-1].total_e;

        let coeffOfRest = lastE / firstE;

        println!("coeff = {:.5}, firstE = {:.5e}, lastE = {:.5e}", coeffOfRest, firstE, lastE);
    }
}


pub fn collisionLots(k: f64, drag: f64) {
    let r: f64 = 1e-7; // 130000 km
    /*
     * diameter = 2e-7
     * area(sq) = 4e-14 
     * 1000 -> 4e-11
     * 1% space covered, aka * 100
     * sqrt( 4e-9 )
     * 6.324555320336759e-05
     */
    let areaAsSquare = (2.*r)*(2.*r);

    let count: i32 = 1000;
    let areaCovered = areaAsSquare * count as f64;

    let fullAreaCovered = areaCovered * 100.;
    let edgeLen = fullAreaCovered.sqrt();

    let cellSize = edgeLen;
    //10e-7;//3e-4;

    // std::cerr << "Survived g.init\n";

    let h: f64 = 1e-5;//1e-6;
    //std::cerr << "> k = " << k << ", drag = " << drag << '\n';
    let mut sys = system::build_system(k, drag);

    // State sys(h, 0, 0, k, drag);

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

    let mut rng = rand::thread_rng();
    
    let root = ((count as f64).sqrt()) as i32;

    let sep = (2.*cellSize) / (root as f64 + 1.);

    for i in 0..root {
        for j in 0..root {
            sys.add_body(
                rng.gen_range(-sep/3., sep/3.) - cellSize + sep * i as f64, 
                rng.gen_range(-sep/3., sep/3.) - cellSize + sep * j as f64, 
                rng.gen_range(-r, r), 
                0.,//rng.gen_range(-1e-7, 1e-7), 
                0.,//rng.gen_range(-1e-7, 1e-7), 
                0.,//rng.gen_range(-1e-7, 1e-7), 
                mass, r);
        }
    }

    // for _ in 0..count {
    //     sys.add_body(
    //         rng.gen_range(-cellSize, cellSize), 
    //         rng.gen_range(-cellSize, cellSize), 
    //         rng.gen_range(-r, r), 
    //         0.,//rng.gen_range(-1e-7, 1e-7), 
    //         0.,//rng.gen_range(-1e-7, 1e-7), 
    //         0.,//rng.gen_range(-1e-7, 1e-7), 
    //         mass, r);
    // }

    const COUNTER_MAX: i32 = 600;

    let mut counter = 0;

    let mut i: f64 = 0.0;
    while i < 100. * PI * 2. * 2.0 * PI { 
        sys.kick_step(h);

        for p in sys.state.iter_mut() {
            system::System::slidingBrickBoundary(p, i, cellSize, cellSize);
        }

        if counter >= 1000 {
            counter = 0;
            eprintln!("{}", i);
            sys.print_out(i);
        }
        counter += 1;
        

        

        i += h;
    }
}