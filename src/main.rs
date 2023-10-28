use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_sys as _;
use std::sync::Arc;

#[macro_use]
pub mod uart;

mod config;
mod console;
pub mod context;
mod control_thread;
mod encoder;
mod fram_logger;
pub mod imu;
mod led;
pub mod misc;
mod motor;
pub mod ods;
mod spiflash;
pub mod timer_interrupt;
mod wall_sensor;

#[allow(unused_imports)]
use led::LedColor::{Blue, Green, Red};

pub static CS: esp_idf_hal::task::CriticalSection = esp_idf_hal::task::CriticalSection::new();

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let ods: Arc<ods::Ods> = Arc::new(ods::Ods::new());

    let mut peripherals = Peripherals::take().unwrap();

    // Initialize LEDs
    let led_tx = led::init(&mut peripherals)?;

    // File system initialization
    spiflash::mount();

    let yaml_config = config::YamlConfig::new()?;
    println!(
        "f32_example = {}",
        yaml_config.load("f32_example")?.as_f64().unwrap() as f32
    );

    motor::init(&mut peripherals)?;
    wall_sensor::init(&mut peripherals)?;
    fram_logger::init(&mut peripherals)?;
    imu::init(&mut peripherals)?;
    encoder::init(&mut peripherals)?;

    timer_interrupt::init(&mut peripherals)?;

    led_tx.send((Green, Some("1")))?;
    led_tx.send((Red, Some("10")))?;
    led_tx.send((Blue, None))?;

    control_thread::init(&yaml_config, &ods)?;

    // You can use println up to before uart:init.
    println!("init uart");
    FreeRtos::delay_ms(100);
    uart::init(&mut peripherals)?;
    // After uart:init, you can use uprintln.
    uprintln!("init uart done");

    let mut console = console::Console::new();
    console.run(&ods);
}
