use esp_idf_hal::delay::FreeRtos;

use crate::context;
use crate::control_thread::{self, ControlThreadCommand};
use crate::imu;
use crate::uart::{read, read_line};
use crate::CS;

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
            Box::new(file::CmdFt {}),
            Box::new(file::CmdShow {}),
            Box::new(file::CmdLs {}),
            Box::new(file::CmdRm {}),
        ];
        Console { commands }
    }

    pub fn run(&mut self) -> ! {
        println!("Welcome to ExtraICE console!");
        loop {
            let mut buf = [0u8; 256];
            let mut args = [""; 16];
            let mut arg_num = 0;
            let mut len = 0;

            uprint!("> ");

            match read_line(&mut buf) {
                Ok(_) => {}
                Err(e) => {
                    uprintln!("Error: {}", e);
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
                    match cmd.execute(&args[1..arg_num]) {
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
    fn execute(&self, args: &[&str]) -> anyhow::Result<()>;
    fn hint(&self);
    fn name(&self) -> &str;
}

/* echo command */
struct CmdEcho {}

/* show all sensor's values */
impl ConsoleCommand for CmdEcho {
    fn execute(&self, args: &[&str]) -> anyhow::Result<()> {
        if args.len() != 0 {
            for arg in args {
                uprintln!("{}", arg);
            }
        } else {
            let mut buffer: [u8; 32] = [0; 32];
            uprint!("Intput : ");
            read_line(&mut buffer)?;
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
    fn execute(&self, args: &[&str]) -> anyhow::Result<()> {
        if args.len() != 0 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }
        {
            let guard = CS.enter();
            context::ope(&guard, |ctx| {
                let _guard = CS.enter();
                ctx.enable_ls = true;
                ctx.enable_lf = true;
                ctx.enable_rf = true;
                ctx.enable_rs = true;
            });
        }

        // Print all sensor data until something is received from UART.
        uprintln!("Press any key to exit.");

        let mut buffer: [u8; 1] = [0];
        loop {
            let mut gyro_yaw = 0.0;
            let ctx: context::Context;
            {
                let guard = CS.enter();
                ctx = context::get();
                unsafe {
                    imu::PHYSICAL.access(&guard, |physical| gyro_yaw = *physical);
                }
            };

            uprintln!(
                "batt: {}, ls: {}, lf: {}, rf: {}, rs: {}, gyro: {}, enc_l: {}, enc_r: {}",
                ctx.batt_raw,
                ctx.ls_raw,
                ctx.lf_raw,
                ctx.rf_raw,
                ctx.rs_raw,
                gyro_yaw,
                ctx.enc_l_raw,
                ctx.enc_r_raw,
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
        {
            let guard = CS.enter();
            context::ope(&guard, |ctx| {
                let _guard = CS.enter();
                ctx.enable_ls = false;
                ctx.enable_lf = false;
                ctx.enable_rf = false;
                ctx.enable_rs = false;
            });
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
    fn execute(&self, _args: &[&str]) -> anyhow::Result<()> {
        control_thread::wait_idle(Some(1000))?;
        control_thread::set_command(ControlThreadCommand::MeasureGyroOffset);
        control_thread::wait_idle(None)?;
        uprintln!("done");
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
