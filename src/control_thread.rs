use crate::led;
use crate::ods;
use crate::timer_interrupt::{sync_ms, wait_us};

use std::sync::Arc;

pub fn init(ods: &Arc<ods::Ods>) -> anyhow::Result<()> {
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: None,
        stack_size: 4096,
        priority: 15,
        inherit: false, // don't inherit this configuration across threads
        pin_to_core: Some(esp_idf_hal::cpu::Core::Core1),
    }
    .set()?;

    let ods = ods.clone();

    std::thread::Builder::new().spawn(move || loop {
        for _ in 0..10 {
            led::toggle(led::LedColor::Blue).unwrap();
            wait_us(50);
        }
        let mut counter = ods.counter.lock().unwrap();
        *counter += 1;
        sync_ms();
    })?;

    Ok(())
}
