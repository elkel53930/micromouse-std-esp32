use std::sync::Mutex;

pub struct Ods {
    pub counter: Mutex<u32>,
}

impl Ods {
    pub fn new() -> Self {
        Ods {
            counter: Mutex::new(0),
        }
    }
}
