use esp_idf_hal::delay::FreeRtos;
use std::sync::{Arc, Mutex};

const DEFAULT_CAPACITY: usize = 10;

pub struct SpinSender<T> {
    buffer: Arc<Mutex<Vec<T>>>,
}

pub struct SpinReceiver<T> {
    buffer: Arc<Mutex<Vec<T>>>,
}

impl<T> SpinSender<T> {
    fn new(buffer: Arc<Mutex<Vec<T>>>) -> Self {
        SpinSender { buffer }
    }

    pub fn send(&self, data: T) {
        let mut buffer = self.buffer.lock().unwrap();
        buffer.push(data);
    }

    pub fn clone(&self) -> Self {
        SpinSender {
            buffer: self.buffer.clone(),
        }
    }
}

impl<T> SpinReceiver<T> {
    fn new(buffer: Arc<Mutex<Vec<T>>>) -> Self {
        SpinReceiver { buffer }
    }

    pub fn recv(&self) -> T {
        self.recv2(1, None).unwrap()
    }

    pub fn recv_timeout(&self, timeout: u32) -> Option<T> {
        self.recv2(1, Some(timeout))
    }

    pub fn try_recv(&self) -> Option<T> {
        let mut buffer = self.buffer.lock().unwrap();
        if buffer.len() > 0 {
            return Some(buffer.remove(0));
        } else {
            return None;
        }
    }

    pub fn recv2(&self, interval: u32, timeout: Option<u32>) -> Option<T> {
        let mut timeout_counter = 0;
        loop {
            match self.try_recv() {
                Some(data) => return Some(data),
                None => (),
            }
            if let Some(timeout) = timeout {
                timeout_counter += interval;
                if timeout_counter >= timeout {
                    return None;
                }
            }
            FreeRtos::delay_ms(interval);
        }
    }
}

pub fn channel<T>() -> (SpinSender<T>, SpinReceiver<T>) {
    let buffer = Arc::new(Mutex::new(Vec::with_capacity(DEFAULT_CAPACITY)));
    (SpinSender::new(buffer.clone()), SpinReceiver::new(buffer))
}
