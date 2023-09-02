use esp_idf_hal::delay::{BLOCK, NON_BLOCK};
use esp_idf_hal::gpio;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::uart::*;

static mut UART: Option<UartDriver> = None;

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    let tx;
    let rx;
    unsafe {
        tx = peripherals.pins.gpio43.clone_unchecked();
        rx = peripherals.pins.gpio44.clone_unchecked();
    }

    let config = config::Config::new().baudrate(Hertz(115_200));
    unsafe {
        let uart = UartDriver::new(
            peripherals.uart1.clone_unchecked(),
            tx,
            rx,
            Option::<gpio::Gpio0>::None,
            Option::<gpio::Gpio1>::None,
            &config,
        )?;

        UART = Some(uart);
    }
    Ok(())
}

#[allow(dead_code)]
pub fn block_read(buf: &mut [u8]) -> Result<usize, esp_idf_sys::EspError> {
    unsafe { UART.as_mut().unwrap().read(buf, BLOCK) }
}

#[allow(dead_code)]
pub fn read(buf: &mut [u8]) -> Result<usize, esp_idf_sys::EspError> {
    unsafe { UART.as_mut().unwrap().read(buf, NON_BLOCK) }
}

#[allow(dead_code)]
pub fn write(buf: &[u8]) -> Result<usize, esp_idf_sys::EspError> {
    unsafe { UART.as_mut().unwrap().write(buf) }
}

use core::fmt::{self, Write};

#[macro_export]
macro_rules! uprint {
    ($($arg:tt)*) => ($crate::uart::_print(format_args!($($arg)*)));
}

#[macro_export]
macro_rules! uprintln {
    ($fmt:expr) => (self::uprint!(concat!($fmt, "\n")));
    ($fmt:expr, $($arg:tt)*) => (self::uprint!(concat!($fmt, "\n"), $($arg)*));
}

pub fn _print(args: fmt::Arguments) {
    let mut writer = UartWriter {};
    writer.write_fmt(args).unwrap();
}

struct UartWriter {}

impl Write for UartWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        write(s.as_bytes()).unwrap();
        Ok(())
    }
}
