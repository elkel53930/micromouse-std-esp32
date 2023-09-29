use crate::config;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::task::CriticalSection;

use crate::{imu, wall_sensor};

static CS: CriticalSection = CriticalSection::new();

pub fn physical_conversion_thread() -> ! {
    loop {
        imu::physical_conversion();
        wall_sensor::physical_conversion();
        FreeRtos::delay_ms(config::CONTROL_CYCLE);
    }
}
