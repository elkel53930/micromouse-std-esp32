use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;

pub enum Command {
    SetDutyCycle(u8),
}

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<Sender<Command>> {
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

    let max_duty = driver.get_max_duty();
    let least_duty = max_duty as f32 / 100.0;

    println!("vac fun, max duty: {}", max_duty);

    thread::spawn(move || {
        let mut pwm_setting_value = 0;
        loop {
            let cmd = rx.recv();
            if cmd.is_ok() {
                match cmd.unwrap() {
                    Command::SetDutyCycle(duty) => {
                        pwm_setting_value = std::cmp::min(duty, 100);
                    }
                }
            }

            let duty = (least_duty * (pwm_setting_value as f32 / 2.0)) as u32;

            driver.set_duty(duty).unwrap();
        }
    });

    Ok(tx)
}
