use esp_idf_hal::{delay::FreeRtos, peripherals::Peripherals};
use esp_idf_sys as _;
use log;
use serde::{Deserialize, Serialize};
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::path::Path;
use std::sync::{
    mpsc::{self, Receiver, Sender},
    {Arc, Mutex},
};

#[macro_use]
pub mod uart;

#[macro_use]
pub mod fram_logger;
use crate::fram_logger::fram_print;

mod console;
mod control_thread;
mod encoder;
pub mod imu;
mod led;
mod led_thread;
mod log_thread;
pub mod misc;
pub mod mm_const;
mod motor;
pub mod ods;
pub mod pid;
mod spiflash;
pub mod timer_interrupt;
mod ui;
mod vac_fan;
mod wall_sensor;

#[allow(unused_imports)]
use led::LedColor::{Blue, Green, Red};

#[derive(Debug, Serialize, Deserialize, Default)]
struct OperationThreadConfig {
    test_config: TestConfig,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct TestConfig {
    test_pattern: Vec<control_thread::Command>,
}

pub struct OperationContext {
    pub ods: Arc<Mutex<ods::Ods>>,
    pub led_tx: Sender<led_thread::Command>,
    pub vac_tx: Sender<vac_fan::Command>,
    pub command_tx: Sender<control_thread::Command>,
    pub response_rx: Receiver<control_thread::Response>,
    pub log_tx: Sender<log_thread::LogCommand>,
}

fn boot_count() -> u32 {
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
        ods: Arc::new(Mutex::new(ods::Ods::new())),
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

    // Count boot times
    let boot_count = boot_count();

    // Start log thread
    let log_tx = log_thread::init(&mut ctx.ods, ctx.led_tx.clone())?;
    ctx.log_tx = log_tx.clone();

    // Initialize peripherals
    motor::init(&mut peripherals)?;
    wall_sensor::init(&mut peripherals)?;
    fram_logger::init(&mut peripherals)?;
    fram_logger::set_log(log::LevelFilter::Info);
    fram_logger::set_panic_handler();
    fram_logger::move_fram_to_flash();
    imu::init(&mut peripherals)?;
    encoder::init(&mut peripherals)?;
    timer_interrupt::init(&mut peripherals)?;

    // Initialize control thread
    let config_success;
    (ctx.command_tx, ctx.response_rx, config_success) =
        control_thread::init(&ctx.ods, log_tx.clone())?;

    // You can use println up to before uart:init.
    FreeRtos::delay_ms(100);
    uart::init(&mut peripherals)?;
    // After uart:init, you can use uprintln.

    let mut config_failure = false;
    if let Err(e) = config_success {
        uprintln!("Config error: {:?}", e);
        log::error!("Config error: {:?}", e);
        config_failure = true;
    }

    // Read config
    fn read() -> anyhow::Result<OperationThreadConfig> {
        let mut f = File::open("/sf/ope_cfg.json")?;
        let mut contents = String::new();
        f.read_to_string(&mut contents)?;
        let result = serde_json::from_str(&contents)?;
        return Ok(result);
    }

    let config = match read() {
        Ok(c) => c,
        Err(e) => {
            uprintln!("❌Failed to read config: {:?}", e);
            config_failure = true;
            OperationThreadConfig::default()
        }
    };

    uprintln!("Boot count: {}", boot_count);
    log::info!("Boot count: {}", boot_count);

    if config_failure {
        ctx.led_tx.send((Red, Some("01")))?;
        ctx.led_tx.send((Blue, Some("01")))?;
        let mut console = console::Console::new();
        console.run(&ctx)?;
    }

    // Initialize done
    app_main(&ctx, config)
}

fn app_main(ctx: &OperationContext, config: OperationThreadConfig) -> anyhow::Result<()> {
    uprintln!("Hold left to enter the console.");

    let mut console = console::Console::new();

    ctx.led_tx.send((Blue, Some("0")))?;
    if ui::hold_ws(&ctx, Some(500)) == ui::UserOperation::HoldR {
        ui::wait(&ctx, ui::UserOperation::HoldL);
        // Calibrate the gyro
        ctx.led_tx.send((Red, Some("10")))?;
        uprintln!("Start gyro calibration");
        ctx.command_tx
            .send(control_thread::Command::GyroCalibration)?;
        let _response = ctx.response_rx.recv().unwrap(); // Wait for Done
        let offset = ctx.ods.lock().unwrap().imu.gyro_x_offset;
        uprintln!("Gyro offset: {}", offset);

        ui::countdown(&ctx);

        for command in config.test_config.test_pattern.iter() {
            log::info!("Sending command: {:?}", command);
            ctx.command_tx.send(*command)?;
            let response = ctx.response_rx.recv()?; // Wait for CommandRequest
            if response != control_thread::Response::CommandRequest {
                return Err(anyhow::anyhow!("Unexpected response: {:?}", response));
            }
        }
        return console.run(&ctx);
    } else {
        // HoldR or TimeOut
        return console.run(&ctx);
    }
}
