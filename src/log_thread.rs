use crate::ods::{self, OdsImu};
use std::fs;
use std::io::Write;
use std::sync::mpsc::{self, Sender};
use std::sync::Arc;

const COUNTER_FILE: &str = "/sf/cnt.txt";
const LOG_FILE_MAX: u32 = 20;

#[derive(Debug, Clone, Copy, Default)]
pub struct Log {
    pub target_x: f32,
    pub current_x: f32,
    pub target_v: f32,
    pub current_v: f32,
    pub current_y: f32,
    pub current_theta: f32,
}

impl Log {
    pub fn to_string(&self) -> String {
        format!(
            "{},{},{},{},{},{}\n",
            self.target_x,
            self.current_x,
            self.target_v,
            self.current_v,
            self.current_y,
            self.current_theta,
        )
    }

    pub fn header() -> String {
        format!("target_x,current_x,target_v,current_v,current_y,current_theta\n")
    }
}

pub const LOG_SIZE: usize = 1000;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LogCommand {
    Save,
}

pub fn init(ods: &Arc<ods::Ods>) -> anyhow::Result<Sender<LogCommand>> {
    let ods = ods.clone();

    /*
       The counter file contains the number of log files used so far.
       The new log file is named logxx.bin, where xx is the number stored in the counter file + 1.
       When the counter reaches LOG_FILE_MAX, it is reset to 0.
       The new value is stored in the counter file.
    */
    let mut counter = 0;
    if let Ok(content) = fs::read_to_string(COUNTER_FILE) {
        if let Ok(value) = content.parse::<u32>() {
            counter = value;
        } else {
            uprintln!("Error parsing counter file, using 0.");
            counter = 0;
        }
    } else {
        uprintln!("Error reading counter file, using 0.");
        counter = 0;
    }
    let filename = format!("/sf/log{:02}.bin", counter);
    counter = (counter + 1) % LOG_FILE_MAX;
    uprintln!("New log file: {}", filename);

    fs::write(COUNTER_FILE, counter.to_string())?;

    match fs::remove_file(&filename) {
        Ok(_) => {
            println!("Old log file {} has been deleted.", filename);
        }
        Err(_) => {}
    }

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
            let command = rx.recv().unwrap();
            if command == LogCommand::Save {
                let mut file = fs::OpenOptions::new()
                    .write(true)
                    .append(true)
                    .open(&filename)?;

                let mut log = ods.log.lock().unwrap();

                uprintln!("[Log] Saving log");

                file.write_all(Log::header().as_bytes())?;

                for log_data in log.iter() {
                    file.write_all(log_data.to_string().as_bytes())?;
                }
                log.clear(); // clear() does not release heap memory.
                uprintln!("[Log] Done");
            }
        }
    })?;

    Ok(tx)
}

pub fn remove_all_logs() -> anyhow::Result<()> {
    for i in 0..LOG_FILE_MAX {
        let filename = format!("/sf/log{:02}.bin", i);
        match fs::remove_file(&filename) {
            Ok(_) => {
                println!("Old log file {} has been deleted.", filename);
            }
            Err(_) => {}
        }
    }
    Ok(())
}