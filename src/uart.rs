use esp_idf_hal::delay::{FreeRtos, NON_BLOCK};
use esp_idf_hal::gpio;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::uart::{self, UartDriver};

static mut UART: Option<UartDriver> = None;

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    let tx;
    let rx;
    unsafe {
        tx = peripherals.pins.gpio43.clone_unchecked();
        rx = peripherals.pins.gpio44.clone_unchecked();
    }

    let config = uart::config::Config::new().baudrate(Hertz(921600));
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

// Read multiple bytes into a slice
pub fn receive(buf: &mut [u8]) -> Result<usize, esp_idf_sys::EspError> {
    unsafe {
        if UART.is_some() {
            UART.as_mut().unwrap().read(buf, NON_BLOCK)
        } else {
            Ok(0)
        }
    }
}

pub fn send(buf: &[u8]) -> Result<usize, esp_idf_sys::EspError> {
    unsafe {
        return UART.as_mut().unwrap().write(buf);
    }
}

// Macros, like println! and print!
use core::fmt::{self, Write};

#[macro_export]
macro_rules! uprint {
    ($($arg:tt)*) => ($crate::uart::_print(format_args!($($arg)*)));
}

#[macro_export]
macro_rules! uprintln {
    ($fmt:expr) => (uprint!(concat!($fmt, "\n")));
    ($fmt:expr, $($arg:tt)*) => (uprint!(concat!($fmt, "\n"), $($arg)*));
}

pub fn _print(args: fmt::Arguments) {
    let mut writer = UartWriter {};
    writer.write_fmt(args).unwrap();
}

pub struct UartWriter {}

impl Write for UartWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        send(s.as_bytes()).unwrap();
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ReadLineResult {
    Ok,
    Escape,
}

// Read until '\n' is received, using read fn.
#[allow(dead_code)]
pub fn read_line(buffer: &mut [u8], escape: bool) -> anyhow::Result<ReadLineResult> {
    let mut i = 0;
    let mut read_buffer: [u8; 1] = [0];
    loop {
        // Read a byte
        match receive(&mut read_buffer) {
            Ok(size) => {
                if size == 0 {
                    FreeRtos::delay_ms(1);
                    continue;
                }
                if read_buffer[0] == b'\n' || read_buffer[0] == b'\r' {
                    // End of line
                    break;
                }
                if escape && read_buffer[0] == 0x1b {
                    // Escape
                    return Ok(ReadLineResult::Escape);
                }
                buffer[i] = read_buffer[0];
                i = i + 1;

                // Buffer full
                if buffer.len() == i {
                    break;
                }
            }
            Err(e) => {
                uprintln!("read_line error: {}", e);
            }
        }
    }
    Ok(ReadLineResult::Ok)
}
