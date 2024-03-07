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
pub mod interpreter;
mod led;
mod led_thread;
mod log_thread;
pub mod misc;
mod motor;
pub mod ods;
pub mod parser;
pub mod pid;
mod spiflash;
pub mod timer_interrupt;
mod trajectory;
mod ui;
mod vac_fan;
mod wall_sensor;

use control_thread::Command;

use mm_maze_solver::maze;
use mm_maze_solver::solver;

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

    // Interpreter test
    {
        let test_script = "let $x = 123.456;
let $s = 0.0;
let $last = 0.0;

if ($x > 1.0){
	$s = $x
} else {
	$s = 1.0
};

let $flag = 1;

while $flag {
	$last = $s;
	$s = ((($x / $s) + $s) / 2.0);
	if ($s >= $last) {
		$flag = 0
	} else {
		pass
	}
};
print!(\"The square root of\", $x, \"is\", $last);
print!($last, \"*\", $last, \"=\", ($last * $last))";

        let parser = parser::p_statement();
        let mut s = parser::ParserState::new(test_script);
        let result = parser(&mut s);
        match result {
            Ok(statement) => {
                let mut env = interpreter::Environment::new();
                let _ = interpreter::evaluate_statement(&mut env, &statement);
            }
            Err(e) => {
                println!("Error at {} : {}", s.pos_string(), e);
            }
        }
    }

    // Stop
    loop {
        esp_idf_hal::delay::FreeRtos::delay_ms(1000);
    }

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
        yaml_config.show();

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

    ctx.led_tx.send((Blue, Some("10")))?;

    // Wait for the user to interrupt the rf sensor
    ctx.command_tx
        .send(control_thread::Command::SetActivateWallSensor(
            true, false, false, true,
        ))?;

    uprintln!("Hold left to enter the console.");

    ctx.led_tx.send((Blue, Some("0")))?;

    let mut console = console::Console::new();
    if ui::hold_ws(&ctx) == ui::UserOperation::HoldL {
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

    ui::countdown(&ctx);

    test_run(&mut ctx)?;

    ctx.led_tx.send((Green, Some("1")))?;

    ctx.led_tx.send((Red, Some("0")))?;

    console.run(&ctx)
}

fn test_run(ctx: &mut OperationContext) -> anyhow::Result<()> {
    use crate::control_thread::Command::*;

    fprintln!("Start test run");

    let mut cmds: Vec<Command> = vec![];
    {
        let yaml_config = config::YamlConfig::new("/sf/config.yaml".to_string())?;
        let test_pattern = yaml_config.load_vec_str("test_pattern", vec![]);

        fprintln!("test_pattern: {:?}", test_pattern);

        for s in test_pattern {
            cmds.push(if s == "StartA" {
                fprintln!("StartA");
                Start(0.017 + 0.045)
            } else if s == "Start" {
                fprintln!("Start");
                Start(0.045)
            } else if s == "Stop" {
                fprintln!("Stop");
                Stop
            } else if s == "Forward" {
                fprintln!("Forward");
                Forward
            } else if s == "TurnL" {
                fprintln!("TurnL");
                TurnL
            } else if s == "TurnR" {
                fprintln!("TurnR");
                TurnR
            } else if s == "TurnBack" {
                fprintln!("TurnBack");
                TurnBack
            } else {
                fprintln!("Unknown command {}", s);
                continue;
            });
        }
    }

    //    let cmds = [Start(0.017 + 0.045), Forward, Stop];
    for cmd in cmds {
        ctx.command_tx.send(cmd)?;
        let _response = ctx.response_rx.recv().unwrap(); // Wait for CommandRequest
    }
    Ok(())
}

fn search_run(ctx: &mut OperationContext) -> anyhow::Result<()> {
    let mut stepmap = solver::StepMap::new();
    let mut local_maze = maze::Maze::new();

    let mut x = 0;
    let mut y = 0;
    let mut d = maze::Direction::East;

    let goal_x = 3;
    let goal_y = 3;

    let mut cmd = Command::Start(0.017 + 0.045);
    while x != goal_x || y != goal_y {
        ctx.command_tx.send(cmd)?;
        let _response = ctx.response_rx.recv().unwrap(); // Wait for CommandRequest
        let wall_sensor = {
            let wall_sensor = ctx.ods.wall_sensor.lock().unwrap();
            wall_sensor.clone()
        };

        fprint!(
            "ls {}({}), lf {}({}), rf {}({}), rs {}({}), ",
            wall_sensor.ls_raw.unwrap(),
            wall_sensor.ls.unwrap(),
            wall_sensor.lf_raw.unwrap(),
            wall_sensor.lf.unwrap(),
            wall_sensor.rf_raw.unwrap(),
            wall_sensor.rf.unwrap(),
            wall_sensor.rs_raw.unwrap(),
            wall_sensor.rs.unwrap()
        );

        if wall_sensor.ls.unwrap() {
            local_maze.set_wall2(y, x, d, maze::Facing::Left, maze::Wall::Present);
        } else {
            local_maze.set_wall2(y, x, d, maze::Facing::Left, maze::Wall::Absent);
        }
        if wall_sensor.rs.unwrap() {
            local_maze.set_wall2(y, x, d, maze::Facing::Right, maze::Wall::Present);
        } else {
            local_maze.set_wall2(y, x, d, maze::Facing::Right, maze::Wall::Absent);
        }
        if wall_sensor.rf.unwrap() || wall_sensor.lf.unwrap() {
            local_maze.set_wall2(y, x, d, maze::Facing::Forward, maze::Wall::Present);
        } else {
            local_maze.set_wall2(y, x, d, maze::Facing::Forward, maze::Wall::Absent);
        }

        let new_dir;
        let dir_to_go =
            match solver::decide_direction(&local_maze, goal_x, goal_y, y, x, &mut stepmap) {
                Some(dir_to_go) => {
                    let (update_x, update_y) = maze::nsew_to_index(dir_to_go);
                    x = ((x as isize) + update_x) as usize;
                    y = ((y as isize) + update_y) as usize;
                    fprint!("x:{}, y:{}, d:{:?}, go:{:?}, ", x, y, d, dir_to_go);
                    new_dir = dir_to_go;
                    maze::nsew_to_fblr(d, dir_to_go)
                }
                None => {
                    fprintln!("Can not reach the goal.");
                    ctx.led_tx.send((Red, Some("0001")))?;
                    break;
                }
            };
        d = new_dir;

        cmd = match dir_to_go {
            maze::DirectionOfTravel::Forward => Command::Forward,
            maze::DirectionOfTravel::Left => Command::TurnL,
            maze::DirectionOfTravel::Right => Command::TurnR,
            maze::DirectionOfTravel::Backward => Command::TurnBack,
        };
        fprintln!("cmd:{:?}", cmd);
    }

    ctx.command_tx.send(Command::Stop)?;

    FreeRtos::delay_ms(2000);

    for line in local_maze.lines_iter(goal_x, goal_y) {
        fprintln!("{}", line);
    }

    Ok(())
}
