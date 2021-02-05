extern crate ncurses;
use std::{thread, time};
use crate::system::System;
use crate::data::basic;
use std::convert::TryInto;


mod graphics_internal {
  pub const DEFAULT_SCALE_FACTOR: f64 = 1.0;

  pub struct ScreenSizeContainer {
    pub max_x: i32,
    pub max_y: i32
  }

  pub fn build_screen_size_container(x: i32, y: i32) -> ScreenSizeContainer {
    return ScreenSizeContainer {
      max_x: x,
      max_y: y
    }
  }
}

pub struct Graphics {
  screen_size: graphics_internal::ScreenSizeContainer,
  initialized: bool,
  scale_factor: f64,
  has_colors: bool
}

pub fn build_graphics() -> Graphics {
  return Graphics {
    screen_size: graphics_internal::ScreenSizeContainer { max_x: 0, max_y: 0 },
    initialized: false,
    scale_factor: graphics_internal::DEFAULT_SCALE_FACTOR,
    has_colors: false
  }
}

pub const WHITE_ON_BLACK: i16 = 4;

impl Graphics {
  pub fn init(&mut self) {
    ncurses::initscr();
    ncurses::clear();
    ncurses::noecho();
    ncurses::cbreak();

    ncurses::curs_set(ncurses::CURSOR_VISIBILITY::CURSOR_INVISIBLE);

    let mut max_y = 0;
    let mut max_x = 0;

    ncurses::getmaxyx(ncurses::stdscr(), &mut max_y, &mut max_x);

    self.screen_size = graphics_internal::build_screen_size_container(max_x, max_y);
    self.initialized = true;

    self.has_colors = ncurses::has_colors();

    if self.has_colors {
      ncurses::start_color();
      
      ncurses::init_pair(0, ncurses::COLOR_YELLOW, ncurses::COLOR_BLACK);
      ncurses::init_pair(1, ncurses::COLOR_CYAN, ncurses::COLOR_BLACK);
      ncurses::init_pair(2, ncurses::COLOR_MAGENTA, ncurses::COLOR_BLACK);
      ncurses::init_pair(3, ncurses::COLOR_RED, ncurses::COLOR_BLACK);
      ncurses::init_pair(WHITE_ON_BLACK, ncurses::COLOR_WHITE, ncurses::COLOR_BLACK);
    }
  }

  pub fn set_scale_factor(&mut self, d: f64) {
    if !self.initialized { panic!("Not initialized"); }

    self.scale_factor = d;
  }

  pub fn draw_point(&mut self, x: f64, y: f64, c: u64, color: i16 ) {
    if !self.initialized { panic!("Not initialized"); }

    let x = x * (self.scale_factor * 3.0 * 5.0/6.0);
    let y = y * self.scale_factor;

    let x = x + (self.screen_size.max_x / 2) as f64;
    let y = y + (self.screen_size.max_y / 2) as f64;

    // std::cerr << "x = " << (int)x << ", y = " << (int)y << '\n';

    if color >= 0 && color < 5 && self.has_colors {
      ncurses::attr_on(ncurses::COLOR_PAIR(color));
    }

    ncurses::mvaddch(y as i32, x as i32, c.try_into().unwrap());

    if color >= 0 && color < 5 && self.has_colors {
      ncurses::attr_off(ncurses::COLOR_PAIR(color));
    }
  }

  pub fn print(&mut self, x: i32, y: i32, s: &str, color: i16 ) {
    if !self.initialized { panic!("Not initialized"); }

    //let x = x * (self.scale_factor * 3.0 * 5.0/6.0);
    //let y = y * self.scale_factor;

    //let x = x + (self.screen_size.max_x / 2) as f64;
    //let y = y + (self.screen_size.max_y / 2) as f64;

    // std::cerr << "x = " << (int)x << ", y = " << (int)y << '\n';

    if color >= 0 && color < 5 && self.has_colors {
      ncurses::attr_on(ncurses::COLOR_PAIR(color));
    }

    ncurses::mvprintw(y, x, s);

    if color >= 0 && color < 5 && self.has_colors {
      ncurses::attr_off(ncurses::COLOR_PAIR(color));
    }
  }

  pub fn end(&mut self) {
    ncurses::endwin();
    self.initialized = false;
  }

  pub fn get_max_x(&self) -> i32 {
    if !self.initialized { panic!("Not initialized"); }

    return self.screen_size.max_x;
  }

  pub fn get_max_y(&self) -> i32 {
    if !self.initialized { panic!("Not initialized"); }

    return self.screen_size.max_y;
  }

  pub fn refresh(&self) {
    if !self.initialized { panic!("Not initialized"); }

    ncurses::refresh();
  }

  pub fn info(&mut self, i: f64, sys: &System) {
    if !self.initialized { panic!("Not initialized"); }

    let msg = format!("Time = {:.3e}", i);
    self.print(0, 0, &msg[..], WHITE_ON_BLACK);
    let v0: basic::Velocity = sys.get_velocity(0);
    let msg = format!("Velocity 1 = <{:.5e}, {:.5e}, {:.5e}>", v0.0, v0.1, v0.2);
    self.print(0, 1, &msg[..], WHITE_ON_BLACK);
  
    // let v1: basic::Velocity = sys.get_velocity(1);
    // let msg2 = format!("Velocity 2 = <{:.5e}, {:.5e}, {:.5e}>", v1.0, v1.1, v1.2);
    // //mvprintw(3, 0, "v 1 = <%e, %e, %e>", v0.x, v0.y, v0.z);
    // self.print(0, 2, &msg2[..], WHITE_ON_BLACK);
  
    let e = sys.energy();
    let msg2 = format!("Energy = {:.5e}", e);
    self.print(0, 3, &msg2[..], WHITE_ON_BLACK);

    let x0: basic::Displacement = sys.get_displacement(0);
    let msg = format!("Displacement 0 = <{:.5e}, {:.5e}, {:.5e}>", x0.0, x0.1, x0.2);
    self.print(0, 4, &msg[..], WHITE_ON_BLACK);
  
    self.print(0, self.get_max_y() - 1, "Press ^C to close", WHITE_ON_BLACK);
  
    self.refresh();
  }

  pub fn centeredBox(&self)
  {
    if !self.initialized { panic!("Not initialized"); }

    let mut max_y = 0;
    let mut max_x = 0;
    ncurses::getmaxyx(ncurses::stdscr(), &mut max_y, &mut max_x);

    let mut min = max_x;
    if max_y < min {
      min = max_y;
    }

    let mid_y = max_y / 2;
    let mid_x = max_x / 2;


    let xMin = (min as f64 * 3.0 * 5.0/6.0) as i32;
    ncurses::mvhline(mid_y - min/2, mid_x - xMin/2, 0, xMin);
    ncurses::mvhline(max_y - 1, mid_x - xMin/2, 0, xMin);
    ncurses::mvvline(mid_y - min/2, mid_x - xMin/2, 0, min);
    ncurses::mvvline(mid_y - min/2, mid_x + xMin/2, 0, min);

    ncurses::mvaddch(mid_y - min/2, mid_x - xMin/2, ncurses::ACS_ULCORNER());
    ncurses::mvaddch(max_y - 1, mid_x - xMin/2, ncurses::ACS_LLCORNER());
    ncurses::mvaddch(mid_y - min/2, mid_x + xMin/2, ncurses::ACS_URCORNER());
    ncurses::mvaddch(max_y - 1, mid_x + xMin/2, ncurses::ACS_LRCORNER());

    self.refresh();
  }

  pub fn clear(&self) {
    if !self.initialized { panic!("Not initialized"); }

    ncurses::clear();
  }
}

pub fn sleep(micros: u64) {
  thread::sleep(time::Duration::from_micros(micros));
}

pub struct GraphicsPoint {
  pub x: f64,
  pub y: f64
}

pub fn build_graphics_point(x: f64, y: f64) -> GraphicsPoint {
  GraphicsPoint { 
    x, 
    y 
  }
}