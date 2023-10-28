use embedded_hal::spi::MODE_3;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{Gpio9, Output, PinDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::{self, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::units::FromValueType;

struct ImuHardware<'a> {
    spi: SpiDeviceDriver<'static, SpiDriver<'static>>,
    chip_select: PinDriver<'a, Gpio9, Output>,
}

// Called from timer interrupt
static mut HARDWARE: Option<ImuHardware<'static>> = None;

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

        HARDWARE = Some(ImuHardware {
            spi: spi,
            chip_select: PinDriver::output(cs)?,
        });
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

    Ok(result)
}
