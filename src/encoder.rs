use embedded_hal::spi::MODE_1;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{Gpio10, Gpio46, Output, PinDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::{self, *};
use esp_idf_hal::units::FromValueType;

use crate::config;

static mut SPI: Option<SpiDeviceDriver<'static, SpiDriver<'static>>> = None;
static mut CS_R: Option<PinDriver<'_, Gpio46, Output>> = None;
static mut CS_L: Option<PinDriver<'_, Gpio10, Output>> = None;

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
        let sda = peripherals.pins.gpio13.clone_unchecked();
        let sdi = peripherals.pins.gpio12.clone_unchecked();
        let cs_r = peripherals.pins.gpio46.clone_unchecked();
        let cs_l = peripherals.pins.gpio10.clone_unchecked();

        let config = spi::config::Config::new()
            .baudrate(5.MHz().into())
            .data_mode(MODE_1);
        let spi = SpiDeviceDriver::new_single(
            spi,
            sclk,
            sda,
            Some(sdi),
            None as Option<Gpio10>,
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

pub fn read_r() -> anyhow::Result<u16> {
    let w_buffer: [u8; 2] = [0x7f, 0xfe];
    let mut r_buffer: [u8; 2] = [0, 0];
    transfer_r(&mut r_buffer, &w_buffer)?;
    Ok(concat(r_buffer[0], r_buffer[1]))
}

pub fn read_l() -> anyhow::Result<u16> {
    let w_buffer: [u8; 2] = [0x7f, 0xfe];
    let mut r_buffer: [u8; 2] = [0, 0];
    transfer_l(&mut r_buffer, &w_buffer)?;
    Ok(concat(r_buffer[0], r_buffer[1]))
}

struct Encoder {
    prev_angle: f32,
}

impl Encoder {
    // Initialize the encoder with an initial angle
    pub fn new(initial_angle: f32) -> Self {
        Encoder {
            prev_angle: initial_angle,
        }
    }

    pub fn to_physical_value(&self, raw: u16) -> f32 {
        2.0 * std::f32::consts::PI * raw as f32 / 16384.0
    }

    pub fn calculate_distance_delta(&mut self, current_raw_angle: u16) -> f32 {
        let current_raw_angle = self.to_physical_value(current_raw_angle);

        // Calculate the change in angle (in radians)
        let mut delta_angle = current_raw_angle - self.prev_angle;

        // Handle overflow
        if delta_angle > std::f32::consts::PI {
            delta_angle -= 2.0 * std::f32::consts::PI;
        } else if delta_angle < -std::f32::consts::PI {
            delta_angle += 2.0 * std::f32::consts::PI;
        }

        // Calculate the distance moved by the wheel
        let distance_moved = (delta_angle / (2.0 * std::f32::consts::PI)) * config::WHEEL_DIAMETER
            / config::GEAR_RATIO;

        // Update the previous angle
        self.prev_angle = current_raw_angle;

        distance_moved
    }

    // Calculate the velocity based on the current angle
    pub fn calculate_velocity(&mut self, distance_moved: f32) -> f32 {
        // Calculate the velocity
        let velocity = distance_moved / (config::CONTROL_CYCLE as f32 / 1000.0);

        velocity
    }

    // Reset the previous angle
    pub fn reset_prev_angle(&mut self, angle: f32) {
        self.prev_angle = angle;
    }
}
