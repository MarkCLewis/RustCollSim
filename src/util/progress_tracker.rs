use indicatif::{MultiProgress, ProgressBar};

pub enum TwoPartProgress {
    Single(ProgressBar),
    Two {
        multi: MultiProgress,
        main_bar: ProgressBar,
        sub_bar: ProgressBar,
    },
}

impl TwoPartProgress {
    pub fn single(bar: ProgressBar) -> Self {
        Self::Single(bar)
    }

    pub fn double(main_bar: ProgressBar, sub_bar: ProgressBar) -> Self {
        let multi = MultiProgress::new();
        let main_bar = multi.add(main_bar);
        let sub_bar = multi.insert_after(&main_bar, sub_bar);

        Self::Two {
            multi,
            main_bar,
            sub_bar,
        }
    }

    pub fn incr_main(&self, delta: u64) {
        match self {
            Self::Single(bar) => bar.inc(delta),
            Self::Two { main_bar, .. } => main_bar.inc(delta),
        }
    }

    pub fn incr_sub(&self, delta: u64) {
        match self {
            Self::Single(_) => {}
            Self::Two { sub_bar, .. } => sub_bar.inc(delta),
        }
    }

    pub fn main_bar(&self) -> &ProgressBar {
        match self {
            Self::Single(bar) => bar,
            Self::Two { main_bar, .. } => main_bar,
        }
    }

    pub fn sub_bar(&self) -> Option<&ProgressBar> {
        match self {
            Self::Single(_) => None,
            Self::Two { sub_bar, .. } => Some(sub_bar),
        }
    }

    pub fn set_sub_message(&self, msg: &'static str) {
        match self {
            Self::Single(_) => {}
            Self::Two { sub_bar, .. } => sub_bar.set_message(msg),
        }
    }
}
