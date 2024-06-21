use crate::led::LedColor::Red;
use crate::led_thread::Command;
use crate::ods;
use std::fs::File;
use std::sync::mpsc::{self, Sender};
use std::sync::{Arc, Mutex};

pub const LOG_SIZE_IN_BYTE: usize = 200_000;
pub const LOG_LEN: usize = LOG_SIZE_IN_BYTE / std::mem::size_of::<crate::ods::MicromouseState>();

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LogCommand {
    Save,
}

pub fn init(
    ods: &Arc<Mutex<ods::Ods>>,
    led_tx: Sender<Command>,
) -> anyhow::Result<Sender<LogCommand>> {
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
            // Wait for the save command
            log::info!("Waiting for the save command...");
            let command = rx.recv().unwrap();
            if command == LogCommand::Save {
                // Write log data as CSV file
                let ods = &mut ods.lock().unwrap();
                log::info!("Saving log data... ({} records)", ods.log.len());

                led_tx.send((Red, Some("1"))).unwrap();
                let mut wtr = csv::Writer::from_writer(File::create("/sf/log.csv")?);
                for log_data in ods.log.iter() {
                    wtr.serialize(log_data)?;
                }
                wtr.flush()?;

                ods.log.clear(); // clear() does not release heap memory.
                log::info!("Saved");
                led_tx.send((Red, Some("0"))).unwrap();
            } else {
                log::warn!("Unknown command: {:?}", command);
            }
        }
    })?;

    Ok(tx) // return the command sender
}
