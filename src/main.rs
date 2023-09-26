use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::timer;
use esp_idf_sys as _;
use std::thread;

#[macro_use]
pub mod uart;

mod config;
mod console;
pub mod context;
pub mod control;
mod control_thread;
mod encoder;
mod fram_logger;
pub mod imu;
mod led;
pub mod misc;
mod motor;
mod spiflash;
pub mod timer_interrupt;
mod wall_sensor;

#[allow(unused_imports)]
use led::LedColor::{Blue, Green, Red};

pub static CS: esp_idf_hal::task::CriticalSection = esp_idf_hal::task::CriticalSection::new();

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let mut peripherals = Peripherals::take().unwrap();

    // Initialize LEDs
    led::init(&mut peripherals)?;
    motor::init(&mut peripherals)?;
    wall_sensor::init(&mut peripherals)?;
    fram_logger::init(&mut peripherals)?;
    imu::init(&mut peripherals)?;
    encoder::init(&mut peripherals)?;

    // You can use println up to before uart:init.
    println!("init uart");
    FreeRtos::delay_ms(100);
    uart::init(&mut peripherals)?;
    // After uart:init, you can use uprintln.
    uprintln!("init uart done");

    // File system initialization
    spiflash::mount();

    config::init();

    uprintln!("f32_example is {}", config::f32_example());

    let timer_config = timer::TimerConfig::new().auto_reload(true);
    let mut timer = timer::TimerDriver::new(peripherals.timer00, &timer_config)?;
    timer.set_alarm(100)?;
    unsafe {
        timer.subscribe(timer_isr)?;
    }

    timer.enable_alarm(true)?;
    timer.enable(true)?;

    // Start LED pattern thread
    thread::spawn(|| loop {
        let _ = led::pattern();
        FreeRtos::delay_ms(100);
    });

    thread::spawn(|| loop {
        imu::physical_conversion();
        FreeRtos::delay_ms(config::CONTROL_CYCLE);
    });

    {
        let guard = CS.enter(); // enter critical section
        context::ope(&guard, |ctx| {
            ctx.enable_ls = true;
            ctx.enable_lf = true;
            ctx.enable_rf = true;
            ctx.enable_rs = true;
        });
        led::set(Green, "1");
        led::set(Red, "10");
    } // end critical section

    thread::spawn(control_thread::control_thread);

    let mut console = console::Console::new();
    console.run();
}

fn timer_isr() {
    timer_interrupt::interrupt().unwrap();
}
