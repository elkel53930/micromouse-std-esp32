use crate::control_thread;
use crate::led::LedColor::*;
use crate::ods;
use crate::uart::{self, read_line, receive};
use crate::OperationContext;
use esp_idf_hal::delay::FreeRtos;
use std::fs::File;
use std::io::prelude::*;

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
        match receive(&mut buffer[i..]) {
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
            Box::new(CmdFprint {}),
            Box::new(CmdFread {}),
            Box::new(CmdPanic {}),
            Box::new(CmdMot {}),
            Box::new(CmdVac {}),
            Box::new(file::CmdFt {}),
            Box::new(file::CmdDl {}),
            Box::new(file::CmdShow {}),
            Box::new(file::CmdLs {}),
            Box::new(file::CmdRm {}),
            Box::new(file::CmdMv {}),
            Box::new(file::CmdLog {}),
        ];
        Console { commands }
    }

    fn list(&self, args: &[&str]) {
        if args.len() != 0 {
            uprintln!("Invalid argument");
            return;
        }

        for cmd in self.commands.iter() {
            uprintln!("{}", cmd.name());
        }
    }

    pub fn run(&mut self, mut ctx: &OperationContext) -> anyhow::Result<()> {
        log::info!("Console started.");
        uprintln!("Welcome to ExtraICE console!");

        loop {
            ctx.led_tx.send((Green, Some("10")))?;
            let mut buf = [0u8; 256];
            let mut args = [""; 16];
            let mut arg_num = 0;
            let mut len = 0;

            let batt_phy = ctx.ods.lock().unwrap().wall_sensor.batt_phy;

            uprint!("{:1.2}[V] > ", batt_phy);

            // receive command
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

            // parse command
            log::info!("Command: {}", std::str::from_utf8(&buf).unwrap());

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
                            log::error!("Command Error: {}", e);
                            cmd.hint();
                        }
                    }
                    ctx.led_tx.send((Red, Some("0")))?;
                    found = true;
                    break;
                }
            }

            if !found {
                match args[0] {
                    "list" => {
                        self.list(&args[1..arg_num]);
                    }
                    "exit" => {
                        uprintln!("Exit console.");
                        log::info!("Exit console.");
                        return Ok(());
                    }
                    _ => {
                        uprintln!("Command not found: '{}'", args[0]);
                        log::info!("Command not found: '{}'", args[0]);
                    }
                }
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
            .send(control_thread::Command::SetActivateWallSensor(true))
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
                let ods = ctx.ods.lock().unwrap();
                batt_raw = ods.wall_sensor.batt_raw;
                ls_raw = ods.wall_sensor.ls_raw;
                lf_raw = ods.wall_sensor.lf_raw;
                rf_raw = ods.wall_sensor.rf_raw;
                rs_raw = ods.wall_sensor.rs_raw;
                gyro_x_raw = ods.imu.gyro_x_raw;
                l_raw = ods.encoder.l;
                r_raw = ods.encoder.r;
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
            match receive(&mut buffer) {
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
            .send(control_thread::Command::SetActivateWallSensor(false))
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

        ctx.ods.lock().unwrap().micromouse = ods::MicromouseState::default();
        uprintln!("Press any key to exit.");
        FreeRtos::delay_ms(500);

        // Print all sensor data until something is received from UART.
        let mut buffer: [u8; 1] = [0];

        loop {
            let (micromouse, gyro) = {
                let ods = ctx.ods.lock().unwrap();
                (ods.micromouse, ods.imu.gyro_x_phy)
            };
            uprintln!(
                "x: {}[m], y: {}[m], theta: {}[rad], gyro: {}[rad/s], v_r: {}[m/s], v_l: {}[m/s]",
                micromouse.x,
                micromouse.y,
                micromouse.theta,
                gyro,
                micromouse.v_r,
                micromouse.v_l
            );
            FreeRtos::delay_ms(100);
            match receive(&mut buffer) {
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
            .send(control_thread::Command::SetActivateWallSensor(false))
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
    fn execute(&self, _args: &[&str], ctx: &OperationContext) -> anyhow::Result<()> {
        FreeRtos::delay_ms(500);
        uprintln!("Calibration...");
        uprintln!("Start gyro calibration");
        ctx.command_tx
            .send(control_thread::Command::GyroCalibration)?;
        let resp = ctx.response_rx.recv().unwrap();
        match resp {
            control_thread::Response::CalibrationDone(offset) => {
                uprintln!("Gyro offset: {}", offset);
            }
            #[allow(unreachable_patterns)]
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
            let mut f = File::open("/sf/ctrl_cfg.json")?;
            let mut contents = String::new();
            f.read_to_string(&mut contents)?;
            let result = serde_json::from_str(&contents)?;
            uprintln!("{:?}", result);
            return Ok(());
        }
        return Err(anyhow::anyhow!("Invalid argument"));
    }

    fn hint(&self) {
        uprintln!("Show current configurations.");
        uprintln!("Usage: config");
    }

    fn name(&self) -> &str {
        "config"
    }
}

struct CmdBatt {}

impl ConsoleCommand for CmdBatt {
    fn execute(&self, args: &[&str], ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() != 0 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        uprintln!("Press any key to exit.");
        FreeRtos::delay_ms(500);

        let mut buffer: [u8; 1] = [0];

        loop {
            let batt_phy = ctx.ods.lock().unwrap().wall_sensor.batt_phy;

            uprintln!("{}[V]", batt_phy);
            FreeRtos::delay_ms(100);
            match receive(&mut buffer) {
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

// Write strings to FRAM using fprintln! macro.
struct CmdFprint {}

impl ConsoleCommand for CmdFprint {
    fn execute(&self, args: &[&str], mut _ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() != 1 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        fprintln!("{}", args[0]);

        return Ok(());
    }

    fn hint(&self) {
        uprintln!("Write strings to FRAM using fprintln! macro.");
        uprintln!("Usage: fprint [string]");
    }

    fn name(&self) -> &str {
        "fprint"
    }
}

// Read strings from FRAM, until the read data is null.
struct CmdFread {}

impl ConsoleCommand for CmdFread {
    fn execute(&self, args: &[&str], mut _ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() != 0 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        let mut buffer: [u8; 32] = [0; 32];
        let mut adrs = 0;
        let mut flag = true;

        while flag {
            crate::fram_logger::read_fram(adrs, &mut buffer)?;
            adrs += buffer.len() as u16;
            for b in buffer {
                if b == 0 {
                    flag = false;
                    break;
                }
                uprint!("{}", b as char);
            }
        }

        return Ok(());
    }

    fn hint(&self) {
        uprintln!("Read strings from FRAM");
        uprintln!("Usage: fread");
    }

    fn name(&self) -> &str {
        "fread"
    }
}

// Intentionally panic.
struct CmdPanic {}

impl ConsoleCommand for CmdPanic {
    fn execute(&self, args: &[&str], mut _ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() != 0 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        let buffer = [0u8; 32];
        #[allow(unconditional_panic)]
        let _x = buffer[50];
        Ok(())
    }

    fn hint(&self) {
        uprintln!("Intentionally panic.");
        uprintln!("Usage: panic");
    }

    fn name(&self) -> &str {
        "panic"
    }
}

// Motor test (set percentage of duty)
struct CmdMot {}

impl ConsoleCommand for CmdMot {
    fn execute(&self, args: &[&str], _ctx: &OperationContext) -> anyhow::Result<()> {
        if args.len() != 2 && args.len() != 0 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        let (l, r) = if args.len() == 0 {
            (0, 0)
        } else {
            (args[0].parse::<i32>()?, args[1].parse::<i32>()?)
        };

        crate::motor::set_l(l as f32);
        crate::motor::set_r(r as f32);

        if l == 0 && r == 0 {
            crate::motor::enable(false);
        } else {
            crate::motor::enable(true);
        }

        Ok(())
    }

    fn hint(&self) {
        uprintln!("Set motor duty.");
        uprintln!("If no argument is specified, stop the motor.");
        uprintln!("Usage: mot [left] [right]");
    }

    fn name(&self) -> &str {
        "mot"
    }
}

// Set vacuum fan duty
struct CmdVac {}

impl ConsoleCommand for CmdVac {
    fn execute(&self, args: &[&str], ctx: &OperationContext) -> anyhow::Result<()> {
        let duty = if args.len() == 1 {
            args[0].parse::<f32>()?
        } else if args.len() == 0 {
            0.0
        } else {
            return Err(anyhow::anyhow!("Invalid argument"));
        };

        if duty > 4.2 {
            return Err(anyhow::anyhow!("The value must be less than 4.2"));
        }

        if duty == 0.0 {
            crate::motor::enable(false);
        } else {
            crate::motor::enable(true);
        }
        ctx.vac_tx.send(crate::vac_fan::Command::SetVoltage(duty))?;

        Ok(())
    }

    fn hint(&self) {
        uprintln!("Set vacuum fan voltage.");
        uprintln!("If no argument is specified, stop the fan.");
        uprintln!("Usage: vac [duty 0 - 4.2]");
    }

    fn name(&self) -> &str {
        "vac"
    }
}
