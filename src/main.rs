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
pub mod misc;
mod motor;
pub mod ods;
pub mod pid;
mod spiflash;
pub mod timer_interrupt;
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

    {
        let yaml_config = config::YamlConfig::new("/sf/config.yaml".to_string())?;
        yaml_config.show();

        motor::init(&mut peripherals)?;
        wall_sensor::init(&mut peripherals)?;
        fram_logger::init(&mut peripherals)?;
        imu::init(&mut peripherals)?;
        encoder::init(&mut peripherals)?;

        timer_interrupt::init(&mut peripherals)?;
        (ctx.command_tx, ctx.response_rx) = control_thread::init(&yaml_config, &ctx.ods)?;
    } // yaml_config is dropped here

    // You can use println up to before uart:init.
    FreeRtos::delay_ms(100);
    uart::init(&mut peripherals)?;
    // After uart:init, you can use uprintln.

    ctx.led_tx.send((Blue, Some("10")))?;

    // Wait for the user to interrupt the rf sensor
    ctx.command_tx
        .send(control_thread::Command::ActivateWallSensor)?;
    loop {
        let rf = {
            let wall_sensor = ctx.ods.wall_sensor.lock().unwrap();
            wall_sensor.rf_raw.unwrap_or(0)
        };
        if rf > 300 {
            break;
        }
    }

    // Calibrate the gyro
    ctx.led_tx.send((Red, Some("10")))?;
    uprintln!("Start gyro calibration");
    ctx.command_tx
        .send(control_thread::Command::GyroCalibration)?;
    let resp = ctx.response_rx.recv().unwrap();
    match resp {
        control_thread::Response::Done => {
            let offset = {
                let imu = ctx.ods.imu.lock().unwrap();
                imu.gyro_x_offset
            };
            uprintln!("Gyro offset: {}", offset);
        }
        _ => {
            uprintln!("Invalid response {:?}", resp);
            panic!("Invalid response {:?}", resp);
        }
    }
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
    ctx.led_tx.send((Red, Some("10")))?;

    ctx.command_tx.send(control_thread::Command::Forward(0.1))?;
    let resp = ctx.response_rx.recv().unwrap();
    match resp {
        control_thread::Response::Done => {
            let x = {
                let micromouse = ctx.ods.micromouse.lock().unwrap();
                micromouse.x
            };
            uprintln!("x: {}", x);
            uprintln!("Done");
        }
        _ => {
            uprintln!("Invalid response {:?}", resp);
            panic!("Invalid response {:?}", resp);
        }
    }
    ctx.led_tx.send((Red, Some("0")))?;

    let mut console = console::Console::new();
    console.run(&ctx)
}
