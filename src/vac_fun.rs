use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{Gpio33, Output, PinDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;

pub enum Command {
    SetDutyCycle(u8),
}

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<Sender<Command>> {
    println!("vac_fun init");
    println!("  init gpio33");
    let mut pwm_pin: PinDriver<'_, Gpio33, Output> =
        unsafe { PinDriver::output(peripherals.pins.gpio33.clone_unchecked()).unwrap() };

    println!("  init command queue");
    let (tx, rx): (Sender<Command>, Receiver<Command>) = mpsc::channel();

    println!("  set thread configuration");
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: None,
        stack_size: 1024,
        priority: 15,
        inherit: false, // don't inherit this configuration across threads
        pin_to_core: Some(esp_idf_hal::cpu::Core::Core0),
    }
    .set()?;

    println!("  spawn thread");
    thread::spawn(move || {
        let mut pwm_duty = 0;
        loop {
            let cmd = rx.try_recv();
            if cmd.is_ok() {
                match cmd.unwrap() {
                    Command::SetDutyCycle(duty) => {
                        pwm_duty = std::cmp::max(duty, 10);
                    }
                }
            }

            if pwm_duty == 0 {
                pwm_pin.set_low().unwrap();
                FreeRtos::delay_ms(10);
            } else if pwm_duty == 10 {
                pwm_pin.set_high().unwrap();
                FreeRtos::delay_ms(10);
            } else {
                pwm_pin.set_high().unwrap();
                FreeRtos::delay_ms(pwm_duty as u32);
                pwm_pin.set_low().unwrap();
                FreeRtos::delay_ms(10 - pwm_duty as u32);
            }
        }
    });

    Ok(tx)
}
