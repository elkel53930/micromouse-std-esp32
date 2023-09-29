use embedded_hal::spi::MODE_3;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{Gpio9, Output, PinDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::{self, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::task::CriticalSection;
use esp_idf_hal::units::FromValueType;

use crate::config;
use crate::context;
use crate::misc::{correct_value, FIR};

struct ImuContext {
    fir: FIR<f32>,
    offset: f32,
}

struct ImuHardware<'a> {
    spi: SpiDeviceDriver<'static, SpiDriver<'static>>,
    chip_select: PinDriver<'a, Gpio9, Output>,
}

// Write by interrupt, read by thread
pub static mut RAW: context::WriteByInterrupt<i16> = context::WriteByInterrupt::Data(0);

// Shared by thread
pub static mut PHYSICAL: context::ShareWithThread<f32> = context::ShareWithThread::Data(0.0);

// Called from timer interrupt
static mut HARDWARE: Option<ImuHardware<'static>> = None;
static mut CONTEXT: Option<ImuContext> = None;
static CS: CriticalSection = CriticalSection::new();

fn transfer(read: &mut [u8], write: &[u8]) -> anyhow::Result<()> {
    unsafe {
        HARDWARE.as_mut().unwrap().chip_select.set_low()?;
    }
    FreeRtos::delay_us(1);
    unsafe {
        HARDWARE.as_mut().unwrap().spi.transfer(read, write)?;
        HARDWARE.as_mut().unwrap().chip_select.set_high()?;
    }
    Ok(())
}

// Called from main
pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        let spi = peripherals.spi2.clone_unchecked();
        let sclk = peripherals.pins.gpio8.clone_unchecked();
        let sda = peripherals.pins.gpio7.clone_unchecked();
        let sdi = peripherals.pins.gpio6.clone_unchecked();
        let cs = peripherals.pins.gpio9.clone_unchecked();
        let config = spi::config::Config::new()
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
        let fir = FIR::new(vec![
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
        ]);

        CONTEXT = Some(ImuContext {
            fir: fir,
            offset: 0.0,
        });

        HARDWARE = Some(ImuHardware {
            spi: spi,
            chip_select: PinDriver::output(cs)?,
        });

        RAW = context::WriteByInterrupt::Data(0);
        PHYSICAL = context::ShareWithThread::Data(0.0);
    }

    let mut r_buffer = [0x00, 0x00];

    let w_buffer = [0x01, 0x80];
    transfer(&mut r_buffer, &w_buffer)?;

    let w_buffer = [0x11, 0xac];
    transfer(&mut r_buffer, &w_buffer)?;

    Ok(())
}

// Called from timer interrupt
pub fn read() -> anyhow::Result<i16> {
    let w_buffer: [u8; 3] = [0xa6, 0xff, 0xff];
    let mut r_buffer: [u8; 3] = [0, 0, 0];
    transfer(&mut r_buffer, &w_buffer)?;

    let result = ((r_buffer[2] as i16) << 8) | (r_buffer[1] as i16);
    unsafe {
        RAW.write(|raw| {
            *raw = result;
        });
    }
    Ok(result)
}

// Called from thread
pub fn get_raw_value() -> i16 {
    unsafe {
        let mut raw_value = 0;
        RAW.read(&context::enter_ics(), |raw| {
            raw_value = *raw;
        });
        raw_value
    }
}

const YAW_TABLE: [(i16, f32); 2] = [(-32768, -2293.76), (32767, 2293.69)];

// Called from thread
pub fn physical_conversion() {
    let (raw_value, offset) = unsafe {
        let raw_value = get_raw_value();
        (raw_value, CONTEXT.as_mut().unwrap().offset)
    };
    let corrected = correct_value(
        &YAW_TABLE,
        raw_value,
        YAW_TABLE[0].1,
        YAW_TABLE[YAW_TABLE.len() - 1].1,
    ) - offset;

    let filtered = unsafe { CONTEXT.as_mut().unwrap().fir.filter(corrected) };

    unsafe {
        let guard = CS.enter();
        PHYSICAL.access(&guard, |physical| {
            *physical = filtered;
        });
    }
}

pub fn get_physical_value() -> f32 {
    unsafe {
        let mut physical_value = 0.0;
        PHYSICAL.access(&CS.enter(), |physical| {
            physical_value = *physical;
        });
        physical_value
    }
}

// Called from thread
pub fn reset_filter() {
    unsafe {
        CONTEXT.as_mut().unwrap().fir.reset();
    }
}

// Called from thread
pub fn measure_offset() {
    let mut sum = 0.0;
    for _ in 0..100 {
        let raw_value = get_raw_value();
        let corrected = correct_value(
            &YAW_TABLE,
            raw_value,
            YAW_TABLE[0].1,
            YAW_TABLE[YAW_TABLE.len() - 1].1,
        );
        sum += corrected as f32;
        FreeRtos::delay_ms(config::CONTROL_CYCLE);
    }
    unsafe {
        CONTEXT.as_mut().unwrap().offset = sum / 100.0;
    }
}
