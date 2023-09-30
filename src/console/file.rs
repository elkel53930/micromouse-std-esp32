/* File transfer command */
pub struct CmdFt {}

use super::blocking_uart_read;
use super::ConsoleCommand;

use esp_idf_hal::delay::FreeRtos;
use std::io::Write;
use std::io::{BufRead, BufReader, Read};

use crate::uart;

impl ConsoleCommand for CmdFt {
    fn execute(&self, args: &[&str]) -> anyhow::Result<()> {
        const CHUNK_SIZE: usize = 256;
        if args.len() != 2 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        let filename = args[0];
        let filesize = match args[1].parse::<u32>() {
            Ok(v) => v,
            Err(e) => return Err(anyhow::anyhow!("Invalid filesize: {}", e)),
        };

        if filesize == 0
        /* todo: check max filesize */
        {
            return Err(anyhow::anyhow!("Invalid filesize {}", filesize));
        }

        // Create a file
        // If the file already exists, remove it
        if std::path::Path::new(filename).exists() {
            std::fs::remove_file(filename)?;
        }
        let mut file = std::fs::File::create(filename)?;

        uprintln!("Start");

        let mut buffer: [u8; CHUNK_SIZE] = [0; CHUNK_SIZE];
        let num_loops = filesize / CHUNK_SIZE as u32;
        let last_chunk_size = filesize % CHUNK_SIZE as u32;

        for _ in 0..num_loops {
            blocking_uart_read(&mut buffer, 10000)?;
            let checksum =
                crc16::State::<crc16::XMODEM>::calculate(&buffer[..last_chunk_size as usize]);
            file.write_all(&buffer)?;
            uprintln!("{:04X}", checksum);
        }

        // Receive the last chunk
        if last_chunk_size != 0 {
            blocking_uart_read(&mut buffer[..last_chunk_size as usize], 10000)?;
            let checksum =
                crc16::State::<crc16::XMODEM>::calculate(&buffer[..last_chunk_size as usize]);
            file.write_all(&buffer[..last_chunk_size as usize])?;
            uprintln!("{:04X}", checksum);
        }

        Ok(())
    }

    fn hint(&self) {
        uprintln!("File transfer");
        uprintln!("Usage: ft {{filename}} {{filesize}}");
        uprintln!("  After issuing the command, the PC sends the data of the file in binary.");
        uprintln!("  The mouse sends back a CRC16-CCITT every 256Byte (chunk).");
        uprintln!("  The file size will not be a multiple of 256Byte in most cases.");
        uprintln!("  So the last chunk will be less than 256Byte.");
    }

    fn name(&self) -> &str {
        "ft"
    }
}

/* Download file */
pub struct CmdDl {}

impl ConsoleCommand for CmdDl {
    fn execute(&self, args: &[&str]) -> anyhow::Result<()> {
        const CHUNK_SIZE: usize = 256;
        if args.len() != 1 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        let filename = args[0];

        if !std::path::Path::new(filename).exists() {
            return Err(anyhow::anyhow!("File not found"));
        }

        FreeRtos::delay_ms(500);

        // Send file size
        let metadata = std::fs::metadata(filename)?;
        let filesize = metadata.len();
        uprintln!("{}", filesize);

        let mut buf: [u8; 10] = [0; 10];
        loop {
            match uart::read(&mut buf) {
                Ok(size) => {
                    if size == 0 {
                        FreeRtos::delay_ms(100);
                        continue;
                    }
                    break;
                }
                Err(e) => {
                    uprintln!("Error: {}", e);
                }
            }
        }
        // Open as binary.
        let mut file = std::fs::File::open(filename)?;
        let mut buffer = [0u8; CHUNK_SIZE];

        while let Ok(size) = file.read(&mut buffer) {
            if size == 0 {
                break;
            }

            uart::write(&buffer[..size])?;
        }

        Ok(())
    }

    fn hint(&self) {
        uprintln!("Download file");
        uprintln!("Usage: dl {{filename}}");
    }

    fn name(&self) -> &str {
        "dl"
    }
}

/* Show file command */
pub struct CmdShow {}

impl ConsoleCommand for CmdShow {
    fn execute(&self, args: &[&str]) -> anyhow::Result<()> {
        if args.len() != 1 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        let filename = args[0];

        if !std::path::Path::new(filename).exists() {
            return Err(anyhow::anyhow!("File not found"));
        }

        let file = std::fs::File::open(filename)?;
        let reader = BufReader::new(file);
        for line in reader.lines() {
            uprintln!("{}", line?);
        }

        Ok(())
    }

    fn hint(&self) {
        uprintln!("Show file contents");
        uprintln!("Usage: show {{filename}}");
    }

    fn name(&self) -> &str {
        "show"
    }
}

/* List files */
pub struct CmdLs {}

impl ConsoleCommand for CmdLs {
    fn execute(&self, args: &[&str]) -> anyhow::Result<()> {
        if args.len() != 1 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        match std::fs::read_dir(args[0]) {
            Ok(entries) => {
                for entry in entries {
                    match entry {
                        Ok(e) => {
                            let path = e.path();
                            if path.is_file() {
                                uprintln!("{}", path.display());
                            }
                        }
                        Err(e) => return Err(anyhow::anyhow!("Error reading entry: {}", e)),
                    }
                }
            }
            Err(e) => return Err(anyhow::anyhow!("Error reading directory: {}", e)),
        }
        Ok(())
    }

    fn hint(&self) {
        uprintln!("List files");
        uprintln!("Usage: ls {{path}}");
    }

    fn name(&self) -> &str {
        "ls"
    }
}

/* Remove file */
pub struct CmdRm {}

impl ConsoleCommand for CmdRm {
    fn execute(&self, args: &[&str]) -> anyhow::Result<()> {
        if args.len() != 1 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        match std::fs::remove_file(args[0]) {
            Ok(_) => {}
            Err(e) => return Err(anyhow::anyhow!("Error removing file: {}", e)),
        }
        Ok(())
    }

    fn hint(&self) {
        uprintln!("Remove file");
        uprintln!("Usage: rm {{filename}}");
    }

    fn name(&self) -> &str {
        "rm"
    }
}

/* Move file */
pub struct CmdMv {}

impl ConsoleCommand for CmdMv {
    fn execute(&self, args: &[&str]) -> anyhow::Result<()> {
        if args.len() != 2 {
            return Err(anyhow::anyhow!("Invalid argument"));
        }

        match std::fs::rename(args[0], args[1]) {
            Ok(_) => {}
            Err(e) => return Err(anyhow::anyhow!("Error moving file: {}", e)),
        }
        Ok(())
    }

    fn hint(&self) {
        uprintln!("Move file");
        uprintln!("Usage: mv {{src}} {{dst}}");
    }

    fn name(&self) -> &str {
        "mv"
    }
}
