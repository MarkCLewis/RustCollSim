extern crate ncurses;
use std::{thread, time};


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

    ncurses::mvaddch(y as i32, x as i32, c);

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
}

pub fn sleep(micros: u64) {
  thread::sleep(time::Duration::from_micros(micros));
}