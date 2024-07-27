use embedded_hal::spi::MODE_1;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{Gpio12, Gpio46, Output, PinDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::{self, *};
use esp_idf_hal::units::FromValueType;

static mut SPI: Option<SpiDeviceDriver<'static, SpiDriver<'static>>> = None;
static mut CS_R: Option<PinDriver<'_, Gpio46, Output>> = None;
static mut CS_L: Option<PinDriver<'_, Gpio12, Output>> = None;

// Transfer data to and from the encoder R
fn transfer_r(read: &mut [u8], write: &[u8]) -> anyhow::Result<()> {
    unsafe {
        CS_R.as_mut().unwrap().set_low()?;
    }
    FreeRtos::delay_us(1);
    unsafe {
        SPI.as_mut().unwrap().transfer(read, write)?;
        CS_R.as_mut().unwrap().set_high()?;
    }
    Ok(())
}

// Transfer data to and from the encoder L
fn transfer_l(read: &mut [u8], write: &[u8]) -> anyhow::Result<()> {
    unsafe {
        CS_L.as_mut().unwrap().set_low()?;
    }
    FreeRtos::delay_us(1);
    unsafe {
        SPI.as_mut().unwrap().transfer(read, write)?;
        CS_L.as_mut().unwrap().set_high()?;
    }
    Ok(())
}

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        let spi = peripherals.spi3.clone_unchecked();
        let sclk = peripherals.pins.gpio11.clone_unchecked();
        let sda = peripherals.pins.gpio9.clone_unchecked();
        let sdi = peripherals.pins.gpio10.clone_unchecked();
        let cs_r = peripherals.pins.gpio46.clone_unchecked();
        let cs_l = peripherals.pins.gpio12.clone_unchecked();

        let config = spi::config::Config::new()
            .baudrate(10.MHz().into())
            .data_mode(MODE_1);
        let spi = SpiDeviceDriver::new_single(
            spi,
            sclk,
            sda,
            Some(sdi),
            None as Option<Gpio12>,
            &SpiDriverConfig::new(),
            &config,
        )?;
        SPI = Some(spi);
        CS_R = Some(PinDriver::output(cs_r)?);
        CS_L = Some(PinDriver::output(cs_l)?);
        CS_R.as_mut().unwrap().set_high()?;
        CS_L.as_mut().unwrap().set_high()?;
    }

    Ok(())
}

fn concat(msb: u8, lsb: u8) -> u16 {
    (msb as u16) * 256 + (lsb as u16) & 0x3fff
}

// Read angular from encoder R
pub fn read_r() -> anyhow::Result<u16> {
    let w_buffer: [u8; 2] = [0x7f, 0xfe];
    let mut r_buffer: [u8; 2] = [0, 0];
    transfer_r(&mut r_buffer, &w_buffer)?;
    Ok(concat(r_buffer[0], r_buffer[1]))
}

// Read angular from encoder L
pub fn read_l() -> anyhow::Result<u16> {
    let w_buffer: [u8; 2] = [0x7f, 0xfe];
    let mut r_buffer: [u8; 2] = [0, 0];
    transfer_l(&mut r_buffer, &w_buffer)?;
    Ok(concat(r_buffer[0], r_buffer[1]))
}
