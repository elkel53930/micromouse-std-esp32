use crate::timer_interrupt::get_time;
use anyhow::Ok;
use esp_idf_hal::delay::BLOCK;
use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::i2c::{I2c, I2cConfig, I2cDriver, Operation};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::prelude::*;
use esp_idf_hal::units::Hertz;

use std::io::Write as _;

const I2C_ADDRESS: u8 = 0x50;

static mut I2C: Option<I2cDriver<'static>> = None;

fn i2c_master_init<'d>(
    i2c: impl Peripheral<P = impl I2c> + 'd,
    sda: AnyIOPin,
    scl: AnyIOPin,
    baudrate: Hertz,
) -> anyhow::Result<I2cDriver<'d>> {
    let config = I2cConfig::new().baudrate(baudrate);
    let driver = I2cDriver::new(i2c, sda, scl, &config)?;
    Ok(driver)
}

fn write_fram(adrs: u16, data: &[u8]) -> anyhow::Result<()> {
    let adrs: [u8; 2] = [(adrs >> 8) as u8, adrs as u8];

    let mut operation = [Operation::Write(&adrs), Operation::Write(data)];

    unsafe {
        I2C.as_mut()
            .unwrap()
            .transaction(I2C_ADDRESS, &mut operation, BLOCK)?;
    }
    Ok(())
}

pub fn read_fram(adrs: u16, data: &mut [u8]) -> anyhow::Result<()> {
    let buffer: [u8; 2] = [(adrs >> 8) as u8, adrs as u8];
    unsafe {
        // Write the address and then read data.
        I2C.as_mut()
            .unwrap()
            .write_read(I2C_ADDRESS, &buffer, data, BLOCK)?;
    }
    Ok(())
}

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        let i2c = i2c_master_init(
            peripherals.i2c0.clone_unchecked(),
            peripherals.pins.gpio14.clone_unchecked().into(),
            peripherals.pins.gpio13.clone_unchecked().into(),
            1000.kHz().into(),
        )?;
        I2C = Some(i2c);
    };
    Ok(())
}

// Macros, like println! and print!
use core::fmt::{self, Write};

static mut CURSOR: u16 = 0; // Write position

pub fn fram_print(args: fmt::Arguments) {
    let mut writer = FramWriter {};
    writer.write_fmt(args).unwrap();
}

#[macro_export]
macro_rules! fprint {
    ($($arg:tt)*) => ($crate::fram_print(format_args!($($arg)*)));
}

#[macro_export]
macro_rules! fprintln {
    ($fmt:expr) => (fprint!(concat!($fmt, "\n")));
    ($fmt:expr, $($arg:tt)*) => (fprint!(concat!($fmt, "\n"), $($arg)*));
}

struct FramWriter;

fn write(s: &str) -> fmt::Result {
    write_fram(unsafe { CURSOR }, s.as_bytes()).unwrap();

    // Advance the cursor
    unsafe {
        CURSOR += s.len() as u16;
        while CURSOR > 0x2000 {
            CURSOR -= 0x2000;
        }
    }

    // Write terminal character
    write_fram(unsafe { CURSOR }, b"\0").unwrap();

    core::result::Result::Ok(())
}

impl Write for FramWriter {
    fn write_str(&mut self, s: &str) -> Result<(), std::fmt::Error> {
        write(&s)
    }
}

use log::{Level, Metadata, Record};
pub struct FramLogger;

impl log::Log for FramLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Info
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let (min, sec, ms, us) = get_time();
            fprintln!(
                "[{:02}:{:02}:{:03}:{:03}] {} - {}",
                min,
                sec,
                ms,
                us,
                record.level(),
                record.args()
            );
        }
    }

    fn flush(&self) {}
}

pub fn set_log(log_level: log::LevelFilter) {
    log::set_boxed_logger(Box::new(FramLogger))
        .map(|()| log::set_max_level(log_level))
        .unwrap();
}

// panic handler with FRAM
use std::panic::{self, PanicInfo};

fn fram_panic_handler(info: &PanicInfo) {
    if let Some(location) = info.location() {
        fprintln!(
            "Panic occurred in file '{}' at line {}",
            location.file(),
            location.line()
        );
    } else {
        fprintln!("Panic occurred but can't get location information...");
    }
    fprintln!("{}", info);

    loop {
        esp_idf_hal::delay::FreeRtos::delay_ms(1000);
    }
}

pub fn set_panic_handler() {
    panic::set_hook(Box::new(fram_panic_handler));
}

pub fn move_fram_to_flash() {
    let mut buffer: [u8; 32] = [0; 32];
    let mut adrs = 0;
    let mut flag = true;

    let _ = std::fs::remove_file("/sf/log05.txt");
    for n in (0..5).rev() {
        let name_from = format!("log{:02}.txt", n);
        let name_to = format!("log{:02}.txt", n + 1);
        let result = std::fs::rename(format!("/sf/{}", name_from), format!("/sf/{}", name_to));
        println!("Move {} to {}: {:?}", name_from, name_to, result);
    }

    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .append(true)
        .open("/sf/log00.txt")
        .unwrap();

    while flag {
        let mut size = 0;
        read_fram(adrs, &mut buffer).unwrap();
        adrs += buffer.len() as u16;
        for b in buffer {
            size += 1;
            if b == 0 {
                flag = false;
                break;
            }
        }
        file.write_all(&buffer[..size]).unwrap();
    }

    println!("Data in FRAM has been copied to log00.txt.");
}
