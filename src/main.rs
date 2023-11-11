use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_sys as _;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;

#[macro_use]
pub mod uart;

mod config;
mod console;
mod control_thread;
mod encoder;
mod fram_logger;
pub mod imu;
mod led;
mod log_thread;
pub mod misc;
mod motor;
pub mod ods;
pub mod pid;
mod spiflash;
pub mod timer_interrupt;
mod trajectory;
mod wall_sensor;

#[allow(unused_imports)]
use led::LedColor::{Blue, Green, Red};

pub static CS: esp_idf_hal::task::CriticalSection = esp_idf_hal::task::CriticalSection::new();

pub struct OperationContext {
    pub ods: Arc<ods::Ods>,
    pub led_tx: Sender<led::Command>,
    pub command_tx: Sender<control_thread::Command>,
    pub response_rx: Receiver<control_thread::Response>,
}

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let mut ctx = OperationContext {
        ods: Arc::new(ods::Ods::new()),
        led_tx: mpsc::channel().0,
        command_tx: mpsc::channel().0,
        response_rx: mpsc::channel().1,
    };

    let mut peripherals = Peripherals::take().unwrap();

    // Initialize LEDs
    ctx.led_tx = led::init(&mut peripherals)?;

    // File system initialization
    spiflash::mount();

    // Start log thread
    let log_tx = log_thread::init(&mut ctx.ods)?;

    {
        let yaml_config = config::YamlConfig::new("/sf/config.yaml".to_string())?;
        yaml_config.show();

        motor::init(&mut peripherals)?;
        wall_sensor::init(&mut peripherals)?;
        fram_logger::init(&mut peripherals)?;
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

    ctx.led_tx.send((Blue, Some("10")))?;

    // Wait for the user to interrupt the rf sensor
    ctx.command_tx
        .send(control_thread::Command::ActivateWallSensor)?;

    let mut goto_console = false;
    loop {
        let (rs, ls) = {
            let wall_sensor = ctx.ods.wall_sensor.lock().unwrap();
            (
                wall_sensor.rs_raw.unwrap_or(0),
                wall_sensor.ls_raw.unwrap_or(0),
            )
        };
        if ls > 1200 {
            goto_console = true;
            break;
        }
        if rs > 1200 {
            break;
        }
    }

    ctx.led_tx.send((Blue, Some("0")))?;

    let mut console = console::Console::new();
    if goto_console {
        return console.run(&ctx);
    }

    // Calibrate the gyro
    ctx.led_tx.send((Red, Some("10")))?;
    uprintln!("Start gyro calibration");
    ctx.command_tx
        .send(control_thread::Command::GyroCalibration)?;
    let _response = ctx.response_rx.recv().unwrap(); // Wait for Done
    let offset = {
        let imu = ctx.ods.imu.lock().unwrap();
        imu.gyro_x_offset
    };
    uprintln!("Gyro offset: {}", offset);
    ctx.led_tx.send((Red, Some("0")))?;

    // Countdown
    ctx.led_tx.send((Green, Some("1")))?;
    FreeRtos::delay_ms(1000);
    ctx.led_tx.send((Green, Some("0")))?;
    ctx.led_tx.send((Blue, Some("1")))?;
    FreeRtos::delay_ms(1000);
    ctx.led_tx.send((Blue, Some("0")))?;
    ctx.led_tx.send((Red, Some("1")))?;
    FreeRtos::delay_ms(1000);
    ctx.led_tx.send((Red, None))?;

    ctx.command_tx
        .send(control_thread::Command::Start(0.017 + 0.045))?;
    let _response = ctx.response_rx.recv().unwrap(); // Wait for CommandRequest

    ctx.command_tx.send(control_thread::Command::Stop)?;
    let _response = ctx.response_rx.recv().unwrap(); // Wait for CommandRequest

    ctx.command_tx.send(control_thread::Command::TurnR)?;
    let _response = ctx.response_rx.recv().unwrap(); // Wait for CommandRequest

    ctx.command_tx.send(control_thread::Command::Stop)?;
    let _response = ctx.response_rx.recv().unwrap(); // Wait for CommandRequest

    ctx.led_tx.send((Green, Some("1")))?;

    ctx.led_tx.send((Red, Some("0")))?;

    console.run(&ctx)
}
