use esp_idf_hal::{delay::FreeRtos, peripherals::Peripherals};
use esp_idf_sys as _;
use log;
use serde::{Deserialize, Serialize};
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::path::Path;
use std::sync::{
    mpsc::{self, Sender},
    {Arc, Mutex},
};

#[macro_use]
pub mod uart;

#[macro_use]
pub mod fram_logger;
use crate::fram_logger::fram_print;

mod console;
mod control_thread;
use control_thread::Command;
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
pub use mm_maze::{adachi, maze, path_finder::PathFinder};
pub mod spin_mpsc;
use spin_mpsc::{SpinReceiver, SpinSender};
pub mod buzzer;

#[allow(unused_imports)]
use led::LedColor::{Blue, Green, Red};

#[derive(Debug, Serialize, Deserialize, PartialEq, Clone, Copy)]
enum OperationMode {
    Search,
    Test,
}

impl Default for OperationMode {
    fn default() -> Self {
        OperationMode::Search
    }
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct SearchConfig {
    goal_x: usize,
    goal_y: usize,
    log_interval: u8,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct OperationThreadConfig {
    mode: OperationMode,
    search_config: SearchConfig,
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
    pub command_tx: SpinSender<Command>,
    pub response_rx: SpinReceiver<control_thread::Response>,
    pub log_tx: Sender<log_thread::LogCommand>,
}

impl OperationContext {
    pub fn wait_response(&self) -> control_thread::Response {
        let response = self.response_rx.recv();
        log::info!("Res : {:?}", response);
        response
    }
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
        command_tx: spin_mpsc::channel().0,
        response_rx: spin_mpsc::channel().1,
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
    buzzer::init(&mut peripherals)?;

    println!("imu::init()");
    imu::init(&mut peripherals)?;
    println!("encoder::init()");
    encoder::init(&mut peripherals)?;
    println!("timer_interrupt::init()");
    timer_interrupt::init(&mut peripherals)?;

    // Initialize control thread
    let config_success;
    (ctx.command_tx, ctx.response_rx, config_success) =
        control_thread::init(&ctx.ods, log_tx.clone())?;

    // You can use println up to before uart:init.
    FreeRtos::delay_ms(500);
    uart::init(&mut peripherals)?;
    // After uart:init, you can use uprintln.
    uprint!("Hello, UART world!\n");

    let mut config_failure = false;
    if let Err(e) = config_success {
        uprintln!("Ope config error: {:?}", e);
        log::error!("Ope config error: {:?}", e);
        config_failure = true;
    }

    uprintln!("Read operation configurations");

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
            uprintln!("❌Failed to read ope config: {:?}", e);
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

    //    ctx.led_tx.send((Blue, Some("0")))?;
    //    if ui::hold_ws(&ctx, Some(500)) == ui::UserOperation::HoldR {
    //        ui::wait(&ctx, ui::UserOperation::HoldL);
    //        // Calibrate the gyro
    //        ctx.led_tx.send((Red, Some("10")))?;
    //        uprintln!("Start gyro calibration");
    //        ctx.command_tx.send(Command::GyroCalibration);
    //        ctx.wait_response(); // Wait for Done
    //        let offset = ctx.ods.lock().unwrap().imu.gyro_x_offset;
    //        uprintln!("Gyro offset: {}", offset);
    //
    //        ui::countdown(&ctx);
    //        if config.mode == OperationMode::Search {
    //            search_run(&ctx, config)?;
    //        } else {
    //            test_run(&ctx, config)?;
    //        }
    //    }
    return console.run(&ctx);
}

fn search_run(ctx: &OperationContext, config: OperationThreadConfig) -> anyhow::Result<()> {
    ctx.led_tx.send((Red, None))?;
    ctx.led_tx.send((Blue, None))?;
    ctx.led_tx.send((Green, None))?;
    let mut maze = maze::Maze::new(16, 16);
    maze.set_goal(maze::Position::new(
        config.search_config.goal_x,
        config.search_config.goal_y,
    ));
    let mut solver = adachi::Adachi::new(maze);

    log::info!(
        "The goal is X:{}, Y:{}",
        solver.get_goal().x,
        solver.get_goal().y
    );

    ctx.command_tx.send(Command::ResetController);
    ctx.wait_response(); // Wait for CommandRequest    ctx.command_tx

    if config.search_config.log_interval != 0 {
        ctx.command_tx
            .send(Command::StartLog(config.search_config.log_interval));
        ctx.wait_response(); // Wait for CommandRequest
    }
    ctx.command_tx.send(Command::SStart(mm_const::BLOCK_LENGTH));
    let mut loc = maze::Location::default();
    loc.forward();
    solver.set_location(loc);
    ctx.wait_response(); // Wait for CommandRequest

    loop {
        let front;
        let left;
        let right;
        {
            let ods = ctx.ods.lock().unwrap();
            let lf_wall = ods.wall_sensor.lf.unwrap();
            let rf_wall = ods.wall_sensor.rf.unwrap();
            front = if lf_wall.to_bool() && rf_wall.to_bool() {
                maze::Wall::Present
            } else {
                maze::Wall::Absent
            };
            left = ods.wall_sensor.ls.unwrap();
            right = ods.wall_sensor.rs.unwrap();
            let ls = ods.wall_sensor.ls_raw.unwrap();
            let rs = ods.wall_sensor.rs_raw.unwrap();
            let lf = ods.wall_sensor.lf_raw.unwrap();
            let rf = ods.wall_sensor.rf_raw.unwrap();
            log::info!("LS: {}, LF: {}, RF: {}, RS: {}", ls, lf, rf, rs);
        }

        let dir = solver.navigate(front, left, right, solver.get_goal());
        if let Err(e) = dir {
            log::warn!("{:?}", e);
            ctx.command_tx.send(Command::SStop);
            ctx.wait_response(); // Wait for CommandRequest
            ctx.command_tx.send(Command::StopLog);
            ctx.wait_response(); // Wait for CommandRequest
            return Ok(());
        }

        // Move to the next location
        let dir = dir.unwrap();

        match dir {
            maze::Direction::Forward => {
                ctx.command_tx.send(Command::SForward);
            }
            maze::Direction::Left => {
                ctx.command_tx.send(Command::SLeft);
            }
            maze::Direction::Right => {
                ctx.command_tx.send(Command::SRight);
            }
            maze::Direction::Backward => {
                ctx.command_tx.send(Command::SReturn);
            }
        }

        let mut loc = solver.get_location();
        loc.dir = loc.dir.turn(dir);
        loc.forward();
        solver.set_location(loc);

        // Check if the goal is reached
        if loc.pos == solver.get_goal() {
            log::info!("Goal reached");
            ctx.command_tx.send(Command::SStop);
            ctx.wait_response(); // Wait for CommandRequest

            break;
        }
        ctx.wait_response(); // Wait for CommandRequest
    }

    if config.search_config.log_interval != 0 {
        ctx.command_tx.send(Command::StopLog);
        ctx.wait_response(); // Wait for CommandRequest
    }
    Ok(())
}

fn test_run(ctx: &OperationContext, config: OperationThreadConfig) -> anyhow::Result<()> {
    for command in config.test_config.test_pattern.iter() {
        log::info!("Sending command: {:?}", command);
        ctx.command_tx.send(*command);
        let response = ctx.wait_response(); // Wait for CommandRequest
        match response {
            control_thread::Response::CommandRequest(_) => {}
            _ => {
                return Err(anyhow::anyhow!("Unexpected response: {:?}", response));
            }
        }
    }
    Ok(())
}
