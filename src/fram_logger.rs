use anyhow::Ok;
use esp_idf_hal::delay::BLOCK;
use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::i2c::{I2c, I2cConfig, I2cDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::prelude::*;
use esp_idf_hal::units::Hertz;

const I2C_ADDRESS: u8 = 0x50;
static mut I2C: Option<I2cDriver<'static>> = None;
static mut CURSOR: u16 = 0;

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
    let mut buffer: [u8; 32] = [0; 32];
    buffer[0] = (adrs >> 8) as u8;
    buffer[1] = adrs as u8;
    buffer[2..2 + data.len()].copy_from_slice(data);
    unsafe {
        I2C.as_mut().unwrap().write(I2C_ADDRESS, &buffer, BLOCK)?;
    }
    Ok(())
}

pub fn read_fram(adrs: u16, data: &mut [u8]) -> anyhow::Result<()> {
    let buffer: [u8; 2] = [(adrs >> 8) as u8, adrs as u8];
    unsafe {
        I2C.as_mut().unwrap().write(I2C_ADDRESS, &buffer, BLOCK)?;
        I2C.as_mut().unwrap().read(I2C_ADDRESS, data, BLOCK)?;
    }
    Ok(())
}

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        let i2c = i2c_master_init(
            peripherals.i2c0.clone_unchecked(),
            peripherals.pins.gpio18.clone_unchecked().into(),
            peripherals.pins.gpio17.clone_unchecked().into(),
            1000.kHz().into(),
        )?;
        I2C = Some(i2c);
    };
    Ok(())
}

use core::fmt::{self, Write};

pub fn fram_print(args: fmt::Arguments) {
    let mut writer = FramWriter {};
    writer.write_fmt(args).unwrap();
}

#[macro_export]
macro_rules! flog {
    ($($arg:tt)*) => ($crate::fram_print(format_args!($($arg)*)));
}

#[macro_export]
macro_rules! flogln {
    ($fmt:expr) => (flog!(concat!($fmt, "\n")));
    ($fmt:expr, $($arg:tt)*) => (flog!(concat!($fmt, "\n"), $($arg)*));
}

struct FramWriter;

fn write(s: &str) -> fmt::Result {
    write_fram(unsafe { CURSOR }, s.as_bytes()).unwrap();
    unsafe {
        CURSOR += s.len() as u16;
        while CURSOR > 0x2000 {
            CURSOR -= 0x2000;
        }
    }
    write_fram(unsafe { CURSOR }, b"\0").unwrap();
    core::result::Result::Ok(())
}

impl Write for FramWriter {
    fn write_str(&mut self, s: &str) -> Result<(), std::fmt::Error> {
        // Write 30 bytes each
        uprintln!("Writing to FRAM {} {}", unsafe { CURSOR }, s);
        let mut i = 0;
        while i < s.len() {
            let mut j = i + 30;
            if j > s.len() {
                j = s.len();
            }
            write(&s[i..j]).unwrap();
            i += 30;
        }
        core::result::Result::Ok(())
    }
}
