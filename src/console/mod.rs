use std::any;

use esp_idf_hal::delay::FreeRtos;

use crate::control_thread;
use crate::led::LedColor::*;
use crate::ods;
use crate::uart::{self, read, read_line};
use crate::OperationContext;

mod file;

fn blocking_uart_read(buffer: &mut [u8], timeout_ms: u32) -> anyhow::Result<()> {
    let size = buffer.len();
    let mut i = 0;
    let mut time_count = 0;
    while i < size {
        if time_count > timeout_ms {
            return Err(anyhow::anyhow!("Receive timeout"));
        }
        time_count += 1;
        FreeRtos::delay_ms(1);
        match read(&mut buffer[i..]) {
            Ok(size) => {
                if size == 0 {
                    continue;
                }
                i += size;
            }
            Err(e) => {
                uprintln!("Error: {}", e);
                continue;
            }
        }
    }
    Ok(())
}

pub struct Console {
    commands: Vec<Box<dyn ConsoleCommand>>,
}

impl Console {
    pub fn new() -> Console {
        let commands: Vec<Box<dyn ConsoleCommand>> = vec![
            Box::new(CmdEcho {}),
            Box::new(CmdSen {}),
            Box::new(CmdOdo {}),
            Box::new(CmdGoffset {}),
            Box::new(CmdReset {}),
            Box::new(CmdConfig {}),
            Box::new(CmdBatt {}),
            Box::new(file::CmdFt {}),
            Box::new(file::CmdDl {}),
            Box::new(file::CmdShow {}),
            Box::new(file::CmdLs {}),
            Box::new(file::CmdRm {}),
            Box::new(file::CmdMv {}),
        ];
        Console { commands }
    }

    pub fn run(&mut self, mut ctx: &OperationContext) -> anyhow::Result<()> {
        println!("Welcome to ExtraICE console!");
        loop {
            ctx.led_tx.send((Green, Some("10")))?;
            let mut buf = [0u8; 256];
            let mut args = [""; 16];
            let mut arg_num = 0;
            let mut len = 0;

            let batt_phy = {
                let wall_sensor = ctx.ods.wall_sensor.lock().unwrap();
                wall_sensor.batt_phy
            };

            uprint!("{:1.2}[V] > ", batt_phy);

            match read_line(&mut buf, true) {
                Ok(result) => {
                    if result == uart::ReadLineResult::Escape {
                        FreeRtos::delay_ms(10);
                        println!("");
                        continue;
                    }
                }
                Err(e) => {
                    uprintln!("{}", e);
                    continue;
                }
            }

            while buf[len] != 0 {
                len += 1;
            }

            if len == 0 {
                continue;
            }

            let mut i = 0;
            let mut arg_start = 0;
            while i < len {
                if buf[i] == ' ' as u8 {
                    args[arg_num] = core::str::from_utf8(&buf[arg_start..i]).unwrap();
                    arg_num += 1;
                    arg_start = i + 1;
                }
                i += 1;
            }
            args[arg_num] = core::str::from_utf8(&buf[arg_start..i]).unwrap();
            arg_num += 1;

            if arg_num == 0 {
                continue;
            }

            let mut found = false;
            for cmd in self.commands.iter_mut() {
                if cmd.name() == args[0] {
                    ctx.led_tx.send((Red, Some("1")))?;
                    match cmd.execute(&args[1..arg_num], &mut ctx) {
                        Ok(_) => {}
                        Err(e) => {
                            uprintln!("Error: {}", e);
                            cmd.hint();
                        }
                    }
                    ctx.led_tx.send((Red, Some("0")))?;
                    found = true;
                    break;
                }
            }
            if !found {
                uprintln!("Command not found: '{}'", args[0]);
            }
        }
    }
}

pub trait ConsoleCommand {
    fn execute(&self, args: &[&str], ctx: &OperationContext) -> anyhow::Result<()>;
    fn hint(&self);
    fn name(&self) -> &str;
}

/* echo command */
struct CmdEcho {}

/* echo arguments */
impl ConsoleCommand for CmdEcho {
    fn execute(&self, args: &[&str], mut _ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() != 0 {
            for arg in args {
                uprintln!("{}", arg);
            }
        } else {
            let mut buffer: [u8; 32] = [0; 32];
            uprint!("Intput : ");
            read_line(&mut buffer, true)?;
            let s = std::str::from_utf8(&buffer).expect("Invalid UTF-8");

            uprintln!("Echo  : '{}'", s);
        }

        Ok(())
    }

    fn hint(&self) {
        uprintln!("Echo input string.");
        uprintln!("Usage: echo [args...]");
    }

    fn name(&self) -> &str {
        "echo"
    }
}

struct CmdSen {}

impl ConsoleCommand for CmdSen {
    fn execute(&self, args: &[&str], ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() != 0 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        // Activate wall sensors
        ctx.command_tx
            .send(control_thread::Command::ActivateWallSensor)
            .unwrap();

        uprintln!("Press any key to exit.");
        FreeRtos::delay_ms(500);

        // Print all sensor data until something is received from UART.
        let mut buffer: [u8; 1] = [0];

        let mut batt_raw;
        let mut ls_raw;
        let mut lf_raw;
        let mut rf_raw;
        let mut rs_raw;
        let mut gyro_x_raw;
        let mut l_raw;
        let mut r_raw;
        loop {
            {
                let imu = ctx.ods.imu.lock().unwrap();
                let encoder = ctx.ods.encoder.lock().unwrap();
                let wall_sensor = ctx.ods.wall_sensor.lock().unwrap();
                batt_raw = wall_sensor.batt_raw;
                ls_raw = wall_sensor.ls_raw;
                lf_raw = wall_sensor.lf_raw;
                rf_raw = wall_sensor.rf_raw;
                rs_raw = wall_sensor.rs_raw;
                gyro_x_raw = imu.gyro_x_raw;
                l_raw = encoder.l;
                r_raw = encoder.r;
            }
            uprintln!(
                "batt: {}, ls: {}, lf: {}, rf: {}, rs: {}, gyro: {}, enc_l: {}, enc_r: {}",
                batt_raw,
                ls_raw.unwrap(),
                lf_raw.unwrap(),
                rf_raw.unwrap(),
                rs_raw.unwrap(),
                gyro_x_raw,
                l_raw,
                r_raw
            );
            FreeRtos::delay_ms(100);
            match read(&mut buffer) {
                Ok(size) => {
                    if size != 0 {
                        break;
                    }
                }
                Err(e) => {
                    uprintln!("Error: {}", e);
                }
            }
        }

        // Inactivate wall sensors
        ctx.command_tx
            .send(control_thread::Command::InactivateWallSensor)
            .unwrap();

        FreeRtos::delay_ms(500);

        println!("");

        Ok(())
    }

    fn hint(&self) {
        uprintln!("Show all sensor's values.");
        uprintln!("Usage: sen");
    }

    fn name(&self) -> &str {
        "sen"
    }
}

struct CmdOdo {}

impl ConsoleCommand for CmdOdo {
    fn execute(&self, args: &[&str], ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() != 0 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        // Activate wall sensors
        ctx.command_tx
            .send(control_thread::Command::ActivateWallSensor)
            .unwrap();

        {
            let mut micromouse = ctx.ods.micromouse.lock().unwrap();
            (*micromouse) = ods::MicromouseState::default();
        }
        uprintln!("Press any key to exit.");
        FreeRtos::delay_ms(500);

        // Print all sensor data until something is received from UART.
        let mut buffer: [u8; 1] = [0];

        loop {
            let micromouse = {
                let micromouse = ctx.ods.micromouse.lock().unwrap();
                *micromouse
            };
            let gyro = {
                let imu = ctx.ods.imu.lock().unwrap();
                imu.gyro_x_phy
            };
            uprintln!(
                "x: {}[m], y: {}[m], theta: {}[rad], gyro: {}[rad/s]",
                micromouse.x,
                micromouse.y,
                micromouse.theta,
                gyro
            );
            FreeRtos::delay_ms(100);
            match read(&mut buffer) {
                Ok(size) => {
                    if size != 0 {
                        break;
                    }
                }
                Err(e) => {
                    uprintln!("Error: {}", e);
                }
            }
        }

        // Inactivate wall sensors
        ctx.command_tx
            .send(control_thread::Command::InactivateWallSensor)
            .unwrap();

        FreeRtos::delay_ms(500);

        println!("");

        Ok(())
    }

    fn hint(&self) {
        uprintln!("Show the micromouse X, Y, Theta.");
        uprintln!("Usage: odo");
    }

    fn name(&self) -> &str {
        "odo"
    }
}

/* goffset cmd */
struct CmdGoffset {}

/* Update gyro offset */
impl ConsoleCommand for CmdGoffset {
    fn execute(&self, _args: &[&str], mut ctx: &OperationContext) -> anyhow::Result<()> {
        FreeRtos::delay_ms(500);
        uprintln!("Calibration...");
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
        Ok(())
    }

    fn hint(&self) {
        uprintln!("Measure the offset of the gyro.");
        uprintln!("Usage: goffset");
    }

    fn name(&self) -> &str {
        "goffset"
    }
}

struct CmdReset {}

impl ConsoleCommand for CmdReset {
    fn execute(&self, args: &[&str], mut _ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() != 0 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }
        unsafe {
            esp_idf_sys::abort();
        }
        #[allow(unreachable_code)]
        Ok(())
    }

    fn hint(&self) {
        uprintln!("Reset the robot.");
        uprintln!("Usage: reset");
    }

    fn name(&self) -> &str {
        "reset"
    }
}

struct CmdConfig {}

impl ConsoleCommand for CmdConfig {
    fn execute(&self, args: &[&str], mut _ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() == 0 {
            let yaml_config = crate::config::YamlConfig::new("/sf/config.yaml".to_string())?;
            yaml_config.ushow();
            return Ok(());
        }

        if args.len() == 1 {
            let yaml_config = crate::config::YamlConfig::new("/sf/config.yaml".to_string())?;
            let value = yaml_config.load(args[0])?;
            uprintln!("{}: {:?}", args[0], value);
            return Ok(());
        }

        return Err(anyhow::anyhow!("Invalid argument"));
    }

    fn hint(&self) {
        uprintln!("Show current configurations.");
        uprintln!("Usage: config [parameter_name]");
        uprintln!("  if parameter_name is not specified, show all parameters.");
    }

    fn name(&self) -> &str {
        "config"
    }
}

struct CmdBatt {}

impl ConsoleCommand for CmdBatt {
    fn execute(&self, args: &[&str], mut _ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() != 0 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        uprintln!("Press any key to exit.");
        FreeRtos::delay_ms(500);

        let mut buffer: [u8; 1] = [0];

        loop {
            let batt_phy = {
                let wall_sensor = _ctx.ods.wall_sensor.lock().unwrap();
                wall_sensor.batt_phy
            };

            uprintln!("{}[V]", batt_phy);
            FreeRtos::delay_ms(100);
            match read(&mut buffer) {
                Ok(size) => {
                    if size != 0 {
                        break;
                    }
                }
                Err(e) => {
                    uprintln!("Error: {}", e);
                }
            }
        }

        return Ok(());
    }

    fn hint(&self) {
        uprintln!("Show the battery voltage.");
        uprintln!("Usage: batt");
    }

    fn name(&self) -> &str {
        "batt"
    }
}
