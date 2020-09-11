#![allow(dead_code)]

mod data;

mod graphics;

use data::data::systems;
use data::data::PI;

fn test_group_2_kick_step_1() {
    let h: f64 = 1e-3;

    let mut sys = systems::build_system();

    sys.push(systems::build_particle(0.,0.,0.,0.,0.,0., 1.0, 1e-7));
    sys.push(systems::build_particle(1.,0.,0.,0.,1.,0.,1e-10, 1e-7));

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

fn test_group_5_graphics(orbits: i32) {
    let h: f64 = 1e-2;

    let mut sys = systems::build_system();

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

        for (c, p) in sys.0.iter().enumerate() {
            g.draw_point(p.location.0, p.location.1, '.' as u64, (c % 4) as i16);
        }

        sys.kick_step(h);

        for (c, p) in sys.0.iter().enumerate() {
            g.draw_point(p.location.0, p.location.1, 'o' as u64, (c % 4) as i16);

        }
        g.refresh();

        graphics::sleep(10000);

        i += h;
    }

    g.end();
}

fn test_group_6_collision() {
    let h = 1e-2;

    let mut sys = systems::build_system();

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

    let mut min: i32 = g.get_max_x();
    if g.get_max_y() < min {
        min = g.get_max_y();
    }

    let factor: f64 = min as f64 / (1e-0 * 2.5);

    g.set_scale_factor(factor);

    let mut i: f64 = 0.0;
    while i < 100. * PI * 2. as f64 * 2.0 * PI {
        //clear();

        for (c, p) in sys.0.iter().enumerate() {
            g.draw_point(p.location.0, p.location.1, '.' as u64, (c % 4) as i16);
        }

        sys.kick_step(h); // uses rk4 but i don't have that rn

        for (c, p) in sys.0.iter().enumerate() {
            g.draw_point(p.location.0, p.location.1, 'o' as u64, (c % 4) as i16);

        }
        g.refresh();

        graphics::sleep(10000);

        i += h;
    }

    g.end();
}

fn main() {
    println!("Hello, world!");

    test_group_6_collision();
    
    //test_group_2_kick_step_1();

}
