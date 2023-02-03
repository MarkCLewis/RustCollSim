// const DO_DEBUG: bool = true;

#[macro_export]
macro_rules! debugln {
    ($s:literal, $( $x:expr ),*) => {
      #[cfg(feature="debug_assertions")]
      {
        let s = format!("{}:{}:{} ", file!(), line!(), column!());
        eprint!("{:50} | ", s);
        eprintln!($s, $(($x),)*);
      }
    };
}
