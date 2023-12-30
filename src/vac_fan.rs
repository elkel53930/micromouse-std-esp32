use crate::ods;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;
use std::thread;

pub enum Command {
    SetVoltage(f32),
}

pub fn init(peripherals: &mut Peripherals, ods: &Arc<ods::Ods>) -> anyhow::Result<Sender<Command>> {
    let ods = ods.clone();

    let timer_driver = unsafe {
        LedcTimerDriver::new(
            peripherals.ledc.timer0.clone_unchecked(),
            &TimerConfig::default().frequency(25.kHz().into()),
        )?
    };
    let mut driver = unsafe {
        LedcDriver::new(
            peripherals.ledc.channel0.clone_unchecked(),
            timer_driver,
            peripherals.pins.gpio33.clone_unchecked(),
        )?
    };
    let (tx, rx): (Sender<Command>, Receiver<Command>) = mpsc::channel();

    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: None,
        stack_size: 1024,
        priority: 15,
        inherit: false, // don't inherit this configuration across threads
        pin_to_core: Some(esp_idf_hal::cpu::Core::Core0),
    }
    .set()?;

    let max_duty = driver.get_max_duty() as f32;

    println!("vac fan, max duty: {}", max_duty);

    thread::spawn(move || {
        loop {
            let mut set_v = 0.0;

            let cmd = rx.recv();
            if cmd.is_ok() {
                match cmd.unwrap() {
                    Command::SetVoltage(v) => {
                        // clip 0.0V - 4.2V
                        set_v = v.min(4.2).max(0.0);
                    }
                }
            }

            let batt_v = ods.wall_sensor.lock().unwrap().batt_phy;
            let pwm_setting_value = (set_v / batt_v * max_duty) as u32;
            driver.set_duty(pwm_setting_value).unwrap();
            FreeRtos::delay_ms(10);
        }
    });

    Ok(tx)
}
