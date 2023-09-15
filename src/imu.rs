use embedded_hal::spi::MODE_3;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{Gpio9, Output, PinDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::*;
use esp_idf_hal::units::FromValueType;

use crate::misc::{correct_value, FIR};

static mut SPI: Option<SpiDeviceDriver<'static, SpiDriver<'static>>> = None;
static mut CS: Option<PinDriver<'_, Gpio9, Output>> = None;
static mut FIR_X: Option<FIR<f32>> = None;
static mut OFFSET_X: i16 = 0;

fn transfer(read: &mut [u8], write: &[u8]) -> anyhow::Result<()> {
    unsafe {
        CS.as_mut().unwrap().set_low()?;
    }
    FreeRtos::delay_us(1);
    unsafe {
        SPI.as_mut().unwrap().transfer(read, write)?;
        CS.as_mut().unwrap().set_high()?;
    }
    Ok(())
}

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        let spi = peripherals.spi2.clone_unchecked();
        let sclk = peripherals.pins.gpio8.clone_unchecked();
        let sda = peripherals.pins.gpio7.clone_unchecked();
        let sdi = peripherals.pins.gpio6.clone_unchecked();
        let cs = peripherals.pins.gpio9.clone_unchecked();
        let config = config::Config::new()
            .baudrate(5.MHz().into())
            .data_mode(MODE_3);
        let spi = SpiDeviceDriver::new_single(
            spi,
            sclk,
            sda,
            Some(sdi),
            None as Option<Gpio9>,
            &SpiDriverConfig::new(),
            &config,
        )?;
        SPI = Some(spi);
        CS = Some(PinDriver::output(cs)?);
        CS.as_mut().unwrap().set_high()?;

        /*
            Filter length : N = 11
            Filter type : LPF
            Window type : Hamming
            Normalized cutoff frequency : 0.4
        */
        FIR_X = Some(FIR::new(vec![
            -2.494829972812401e-18,
            -7.851195903558143e-03,
            4.014735544403485e-02,
            -1.032535402203297e-01,
            1.706609016841135e-01,
            8.000000000000000e-01,
            1.706609016841135e-01,
            -1.032535402203297e-01,
            4.014735544403485e-02,
            -7.851195903558143e-03,
            -2.494829972812401e-18,
        ]));
    }

    let mut r_buffer = [0x00, 0x00];

    let w_buffer = [0x01, 0x80];
    transfer(&mut r_buffer, &w_buffer)?;

    let w_buffer = [0x11, 0xac];
    transfer(&mut r_buffer, &w_buffer)?;

    Ok(())
}

pub fn read() -> anyhow::Result<i16> {
    let w_buffer: [u8; 3] = [0xa6, 0xff, 0xff];
    let mut r_buffer: [u8; 3] = [0, 0, 0];
    transfer(&mut r_buffer, &w_buffer)?;

    let result = ((r_buffer[2] as i16) << 8) | (r_buffer[1] as i16);
    Ok(result)
}

const YAW_TABLE: [(i16, f32); 2] = [(-32768, -2293.76), (32767, 2293.69)];

pub fn correct(raw_value: i16) -> f32 {
    let offset_removed = raw_value - unsafe { OFFSET_X };
    let corrected = correct_value(
        &YAW_TABLE,
        offset_removed,
        YAW_TABLE[0].1,
        YAW_TABLE[YAW_TABLE.len() - 1].1,
    );
    let filtered = unsafe { FIR_X.as_mut().unwrap().filter(corrected) };
    filtered
}
