use std::sync::{Arc, Mutex};

pub struct Asserv {

}

impl Asserv {
    pub fn new() -> Arc<Mutex<Self>> {
        Arc::new(Mutex::new(Self {

        }))
    }
}