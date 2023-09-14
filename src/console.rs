use crate::uart::read_line;

pub struct Console {
    commands: Vec<Box<dyn ConsoleCommand>>,
}

impl Console {
    pub fn new() -> Console {
        let commands: Vec<Box<dyn ConsoleCommand>> = vec![Box::new(CmdEcho {})];
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

/* sen command */
pub struct CmdEcho {}

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
        uprintln!("Usage: echo [args...]");
    }

    fn name(&self) -> &str {
        "echo"
    }
}
