#![allow(dead_code)]
#![allow(non_snake_case)]

mod data;
mod graphics;
mod system;
mod collision;
mod sim1D;
mod fourthOrderInt;

use data::PI;


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
    sys.print_out(-1.);
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
    sys.print_out(-1.);
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

        g.info(i, &sys);

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

mod no_explode;

mod sim1D_piecewise;

fn main() {
    eprintln!("Hello, world!");

    fourthOrderInt::main();

    return;

    // let v0 = 0.0000001;
    // let m = 1.0266666666666665e-20;

    // let (b,k) = compute::b_and_k(v0, m);
    // eprintln!("b = {:e}, k = {:e}", b, k);

    //sim1D::testing2();

    sim1D_piecewise::testing2();

    //test_group_5_graphics(2);
    //test_group_3_rk4_1();
    //test_group_6_collision();

    // collision::collisionLots(1e-8, 5e-15);
    
    // let output = collision::collision_no_graphics(1e-8, 1e-15, 10);
    // output.print_out_json();
    
    //test_group_2_kick_step_1();

    // let drag = vec!(1e-15, 2e-15, 3e-15, 4e-15, 5e-15);
    // let k = 1e-8;

    // collision::collision_drag_analysis(k, drag);

}
