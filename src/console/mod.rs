use esp_idf_hal::delay::FreeRtos;
use esp_idf_sys::_WANT_REENT_SMALL;

use crate::ods;
use crate::uart::{self, read, read_line};

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
            Box::new(CmdGoffset {}),
            Box::new(CmdReset {}),
            Box::new(file::CmdFt {}),
            Box::new(file::CmdDl {}),
            Box::new(file::CmdShow {}),
            Box::new(file::CmdLs {}),
            Box::new(file::CmdRm {}),
            Box::new(file::CmdMv {}),
        ];
        Console { commands }
    }

    pub fn run(&mut self, mut ods: &ods::Ods) -> ! {
        println!("Welcome to ExtraICE console!");
        loop {
            let mut buf = [0u8; 256];
            let mut args = [""; 16];
            let mut arg_num = 0;
            let mut len = 0;

            uprint!("> ");

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
                    match cmd.execute(&args[1..arg_num], &mut ods) {
                        Ok(_) => {}
                        Err(e) => {
                            uprintln!("Error: {}", e);
                            cmd.hint();
                        }
                    }
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
    fn execute(&self, args: &[&str], ods: &ods::Ods) -> anyhow::Result<()>;
    fn hint(&self);
    fn name(&self) -> &str;
}

/* echo command */
struct CmdEcho {}

/* show all sensor's values */
impl ConsoleCommand for CmdEcho {
    fn execute(&self, args: &[&str], mut _ods: &ods::Ods) -> anyhow::Result<()> {
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
    fn execute(&self, args: &[&str], mut ods: &ods::Ods) -> anyhow::Result<()> {
        if args.len() != 0 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        uprintln!("Press any key to exit.");
        FreeRtos::delay_ms(500);

        // Print all sensor data until something is received from UART.
        let mut buffer: [u8; 1] = [0];

        let mut batt_raw = 0;
        let mut ls_raw = 0;
        let mut lf_raw = 0;
        let mut rf_raw = 0;
        let mut rs_raw = 0;
        let mut gyro_x_raw = 0;
        let mut l_raw = 0;
        let mut r_raw = 0;
        loop {
            {
                let imu = ods.imu.lock().unwrap();
                let encoder = ods.encoder.lock().unwrap();
                let wall_sensor = ods.wall_sensor.lock().unwrap();
                batt_raw = wall_sensor.batt_raw;
                ls_raw = wall_sensor.ls_raw;
                lf_raw = wall_sensor.lf_raw;
                rf_raw = wall_sensor.rf_raw;
                rs_raw = wall_sensor.rs_raw;
                gyro_x_raw = imu.gyro_x_raw;
                l_raw = encoder.l_raw;
                r_raw = encoder.r_raw;
            }
            uprintln!(
                "batt: {}, ls: {}, lf: {}, rf: {}, rs: {}, gyro: {}, enc_l: {}, enc_r: {}",
                batt_raw,
                ls_raw,
                lf_raw,
                rf_raw,
                rs_raw,
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

/* goffset cmd */
struct CmdGoffset {}

/* show all sensor's values */
impl ConsoleCommand for CmdGoffset {
    fn execute(&self, _args: &[&str], mut _ods: &ods::Ods) -> anyhow::Result<()> {
        FreeRtos::delay_ms(500);
        uprintln!("Calibration...");
        //        uprintln!("offset is {}", imu::measure_offset(1000));
        uprintln!("NOT IMPLEMENTED YET!");
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
    fn execute(&self, args: &[&str], mut _ods: &ods::Ods) -> anyhow::Result<()> {
        if args.len() != 0 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }
        unsafe {
            esp_idf_sys::abort();
        }
    }

    fn hint(&self) {
        uprintln!("Reset the robot.");
        uprintln!("Usage: reset");
    }

    fn name(&self) -> &str {
        "reset"
    }
}
