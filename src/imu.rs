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

static mut HARDWARE: Option<ImuHardware<'static>> = None;

/*
    Write command:
        1st byte |   0 | AD6 | AD5 | AD4 | AD3 | AD2 | AD1 | AD0 |
        2nd byte |                 Wite data                     |
                                       :

    Read command:
        1st byte |   1 | AD6 | AD5 | AD4 | AD3 | AD2 | AD1 | AD0 |
        2nd byte |                 Read data                     |
                                       :
*/

// Transfer data to and from the IMU
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

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        let spi = peripherals.spi2.clone_unchecked();
        let sclk = peripherals.pins.gpio8.clone_unchecked();
        let sdo = peripherals.pins.gpio7.clone_unchecked();
        let sdi = peripherals.pins.gpio6.clone_unchecked();
        let cs = peripherals.pins.gpio9.clone_unchecked();
        let config = spi::config::Config::new()
            .baudrate(5.MHz().into())
            .data_mode(MODE_3);
        let spi = SpiDeviceDriver::new_single(
            spi,
            sclk,
            sdo,
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

    // Set 1 to FUNC_CFG_ACCES bit in FUNC_CFG_ACCESS (01h) register,
    // to enable writing to configuration registers
    let w_buffer = [0x01, 0x80];
    transfer(&mut r_buffer, &w_buffer)?;

    // Configure CTRL2_G (11h) register
    //   Data rate : 6.66kHz (high performance)
    //   Full scale : +/-2000[dps]
    let w_buffer = [0x11, 0xac];
    transfer(&mut r_buffer, &w_buffer)?;

    Ok(())
}

pub fn read() -> anyhow::Result<i16> {
    // Read OUTZ_L_G (26h) and OUTZ_H_G (27h)
    let w_buffer: [u8; 3] = [0xa6, 0xff, 0xff]; // Read 2 bytes from 0x26
    let mut r_buffer: [u8; 3] = [0, 0, 0];
    transfer(&mut r_buffer, &w_buffer)?;

    let result = ((r_buffer[2] as i16) << 8) | (r_buffer[1] as i16);

    Ok(result)
}
