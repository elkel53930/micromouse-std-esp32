use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_sys as _;
use log;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;

#[macro_use]
pub mod uart;

#[macro_use]
pub mod fram_logger;
use crate::fram_logger::fram_print;

mod config;
mod console;
mod control_thread;
mod encoder;
pub mod imu;
mod led;
mod led_thread;
mod log_thread;
pub mod misc;
mod motor;
pub mod ods;
mod spiflash;
pub mod timer_interrupt;
mod vac_fan;
mod wall_sensor;

#[allow(unused_imports)]
use led::LedColor::{Blue, Green, Red};

pub static CS: esp_idf_hal::task::CriticalSection = esp_idf_hal::task::CriticalSection::new();

pub struct OperationContext {
    pub ods: Arc<ods::Ods>,
    pub led_tx: Sender<led_thread::Command>,
    pub vac_tx: Sender<vac_fan::Command>,
    pub command_tx: Sender<control_thread::Command>,
    pub response_rx: Receiver<control_thread::Response>,
    pub log_tx: Sender<log_thread::LogCommand>,
}

fn boot_count() -> u32 {
    use std::fs::{File, OpenOptions};
    use std::io::{Read, Write};
    use std::path::Path;
    let path = Path::new("/sf/boot_count");

    let count = if path.exists() {
        let mut file = File::open(path).unwrap();
        let mut contents = String::new();
        file.read_to_string(&mut contents).unwrap();
        contents.trim().parse::<u32>().unwrap_or(0)
    } else {
        0
    };

    let mut file = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open(path)
        .unwrap();
    write!(file, "{}", count + 1).unwrap();

    count
}

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let mut ctx = OperationContext {
        ods: Arc::new(ods::Ods::new()),
        led_tx: mpsc::channel().0,
        command_tx: mpsc::channel().0,
        response_rx: mpsc::channel().1,
        log_tx: mpsc::channel().0,
        vac_tx: mpsc::channel().0,
    };

    let mut peripherals = Peripherals::take().unwrap();

    // Initialize LEDs
    led::init(&mut peripherals)?;
    ctx.led_tx = led_thread::init()?;

    // Initialize vacuum motor
    ctx.vac_tx = vac_fan::init(&mut peripherals, &mut ctx.ods)?;

    // File system initialization
    spiflash::mount();

    let boot_count = boot_count();

    // Start log thread
    let log_tx = log_thread::init(&mut ctx.ods)?;
    ctx.log_tx = log_tx.clone();

    {
        let yaml_config = config::YamlConfig::new("/sf/config.yaml".to_string())?;

        motor::init(&mut peripherals)?;
        wall_sensor::init(&mut peripherals)?;
        fram_logger::init(&mut peripherals)?;
        fram_logger::set_log(log::LevelFilter::Info);
        fram_logger::set_panic_handler();
        fram_logger::move_fram_to_flash();
        imu::init(&mut peripherals)?;
        encoder::init(&mut peripherals)?;

        timer_interrupt::init(&mut peripherals)?;
        (ctx.command_tx, ctx.response_rx) =
            control_thread::init(&yaml_config, &ctx.ods, log_tx.clone())?;
    } // yaml_config is dropped here

    // You can use println up to before uart:init.
    FreeRtos::delay_ms(100);
    uart::init(&mut peripherals)?;
    // After uart:init, you can use uprintln.

    uprintln!("Boot count: {}", boot_count);
    fprintln!("Boot count: {}", boot_count);

    // Wait for the user to interrupt the rf sensor
    ctx.command_tx
        .send(control_thread::Command::SetActivateWallSensor(
            true, false, false, true,
        ))?;

    ctx.led_tx.send((Blue, Some("0")))?;

    let mut console = console::Console::new();

    console.run(&ctx)
}
