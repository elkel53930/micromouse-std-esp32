use crate::ods;
use std::fs;
use std::io::Write;
use std::sync::mpsc::{self, Sender};
use std::sync::Arc;

const LOG_FILE_NAME: &str = "/sf/log.csv";

#[derive(Debug, Clone, Copy, Default)]
pub struct Log {
    pub current_ms: u32,
    pub target_x: f32,
    pub current_x: f32,
    pub target_v: f32,
    pub current_v: f32,
    pub current_y: f32,
    pub current_theta: f32,
    pub motor_l: f32,
    pub motor_r: f32,
    pub ff_ctrl: f32,
    pub position_x_ctrl: f32,
    pub position_y_ctrl: f32,
    pub omega_ctrl: f32,
}

impl Log {
    pub fn to_string(&self) -> String {
        format!(
            "{},{},{},{}\n",
            self.target_x, self.current_x, self.target_v, self.current_v,
        )
    }

    pub fn header() -> String {
        format!("target_x,current_x,target_v,current_v\n")
    }
}

pub const LOG_SIZE: usize = 500;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LogCommand {
    Start,
    Save,
}

pub fn init(ods: &Arc<ods::Ods>) -> anyhow::Result<Sender<LogCommand>> {
    let ods = ods.clone();

    // Spawn the log thread
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: None,
        stack_size: 2048,
        priority: 15,
        inherit: false, // don't inherit this configuration across threads
        pin_to_core: Some(esp_idf_hal::cpu::Core::Core0),
    }
    .set()?;

    /* mpsc */
    let (tx, rx) = mpsc::channel::<LogCommand>();

    std::thread::Builder::new().spawn(move || -> anyhow::Result<()> {
        loop {
            loop {
                let command = rx.recv().unwrap();
                if command == LogCommand::Start {
                    uprintln!("[Log] Started");
                    let _ = fs::remove_file(LOG_FILE_NAME);
                    break;
                }
            }
            loop {
                let command = rx.recv().unwrap();
                if command == LogCommand::Save {
                    let mut file = fs::OpenOptions::new()
                        .write(true)
                        .append(true)
                        .open(LOG_FILE_NAME)?;

                    let mut log = ods.log.lock().unwrap();

                    uprintln!("[Log] Saving log");

                    file.write_all(Log::header().as_bytes())?;

                    let mut counter = 0;

                    for log_data in log.iter() {
                        file.write_all(log_data.to_string().as_bytes())?;
                        file.flush()?;
                        counter += 1;
                    }

                    uprintln!("counter: {}", counter);

                    log.clear(); // clear() does not release heap memory.
                    uprintln!("[Log] Done");
                    break;
                }
            }
        }
    })?;

    Ok(tx)
}
