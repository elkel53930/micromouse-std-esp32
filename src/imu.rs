use embedded_hal::spi::MODE_3;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{
    self, Gpio1, Gpio10, Gpio11, Gpio12, Gpio2, Gpio3, Gpio4, Gpio46, Gpio5, Gpio6, Output,
    PinDriver,
};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::{self, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::units::FromValueType;
use esp_idf_sys::gpio_set_level;

static CS_RF: i32 = 46;
static EN_LF: i32 = 12;
static CS_LF: i32 = 11;
static CS_BT: i32 = 6;
static EN_LS: i32 = 5;
static CS_LS: i32 = 4;
static EN_RS: i32 = 3;
static CS_RS: i32 = 2;
static EN_RF: i32 = 1;

struct ImuHardware<'a> {
    spi: SpiDeviceDriver<'static, SpiDriver<'static>>,
    chip_select: PinDriver<'a, Gpio10, Output>,
    cs_bt: PinDriver<'a, Gpio6, Output>,
    cs_lf: PinDriver<'a, Gpio11, Output>,
    cs_ls: PinDriver<'a, Gpio4, Output>,
    cs_rs: PinDriver<'a, Gpio2, Output>,
    cs_rf: PinDriver<'a, Gpio46, Output>,

    en_lf: PinDriver<'a, Gpio12, Output>,
    en_ls: PinDriver<'a, Gpio5, Output>,
    en_rs: PinDriver<'a, Gpio3, Output>,
    en_rf: PinDriver<'a, Gpio1, Output>,
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
fn imu_transfer(read: &mut [u8], write: &[u8]) -> anyhow::Result<()> {
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
        let sclk = peripherals.pins.gpio9.clone_unchecked();
        let sdo = peripherals.pins.gpio8.clone_unchecked();
        let sdi = peripherals.pins.gpio7.clone_unchecked();
        let cs = peripherals.pins.gpio10.clone_unchecked();
        let cs_batt = peripherals.pins.gpio6.clone_unchecked();
        let cs_lf = peripherals.pins.gpio11.clone_unchecked();
        let cs_ls = peripherals.pins.gpio4.clone_unchecked();
        let cs_rs = peripherals.pins.gpio2.clone_unchecked();
        let cs_rf = peripherals.pins.gpio46.clone_unchecked();
        let en_lf = peripherals.pins.gpio12.clone_unchecked();
        let en_ls = peripherals.pins.gpio5.clone_unchecked();
        let en_rs = peripherals.pins.gpio3.clone_unchecked();
        let en_rf = peripherals.pins.gpio1.clone_unchecked();

        let config = spi::config::Config::new()
            .baudrate(5.MHz().into())
            .data_mode(MODE_3);
        let spi = SpiDeviceDriver::new_single(
            spi,
            sclk,
            sdo,
            Some(sdi),
            None as Option<Gpio10>,
            &SpiDriverConfig::new(),
            &config,
        )?;

        HARDWARE = Some(ImuHardware {
            spi: spi,
            chip_select: PinDriver::output(cs)?,
            cs_bt: PinDriver::output(cs_batt)?,
            cs_lf: PinDriver::output(cs_lf)?,
            cs_ls: PinDriver::output(cs_ls)?,
            cs_rs: PinDriver::output(cs_rs)?,
            cs_rf: PinDriver::output(cs_rf)?,
            en_lf: PinDriver::output(en_lf)?,
            en_ls: PinDriver::output(en_ls)?,
            en_rs: PinDriver::output(en_rs)?,
            en_rf: PinDriver::output(en_rf)?,
        });
    }

    off_lf()?;
    off_ls()?;
    off_rs()?;
    off_rf()?;

    let mut r_buffer = [0x00, 0x00];

    // Set 1 to FUNC_CFG_ACCES bit in FUNC_CFG_ACCESS (01h) register,
    // to enable writing to configuration registers
    let w_buffer = [0x01, 0x80];
    imu_transfer(&mut r_buffer, &w_buffer)?;

    // Configure CTRL2_G (11h) register
    //   Data rate : 6.66kHz (high performance)
    //   Full scale : +/-2000[dps]
    let w_buffer = [0x11, 0xac];
    imu_transfer(&mut r_buffer, &w_buffer)?;

    Ok(())
}

pub fn read() -> anyhow::Result<i16> {
    // Read OUTZ_L_G (26h) and OUTZ_H_G (27h)
    let w_buffer: [u8; 3] = [0xa6, 0xff, 0xff]; // Read 2 bytes from 0x26
    let mut r_buffer: [u8; 3] = [0, 0, 0];
    imu_transfer(&mut r_buffer, &w_buffer)?;

    let result = ((r_buffer[2] as i16) << 8) | (r_buffer[1] as i16);

    Ok(result)
}

pub fn read_batt() -> anyhow::Result<u16> {
    let write = [0xff, 0xff];
    let mut read = [0, 0];
    unsafe {
        let hw = HARDWARE.as_mut().unwrap();
        gpio_set_level(CS_BT, 0);
        hw.spi.transfer(&mut read, &write)?;
        gpio_set_level(CS_BT, 1);
    }

    let result = ((read[0] as u16) << 8) | read[1] as u16;
    let result = result & 0x1fff;
    let result = result / 2;

    Ok(result)
}

pub fn read_lf() -> anyhow::Result<u16> {
    let write = [0xff, 0xff];
    let mut read = [0, 0];
    unsafe {
        let hw = HARDWARE.as_mut().unwrap();
        gpio_set_level(CS_LF, 0);
        hw.spi.transfer(&mut read, &write)?;
        gpio_set_level(CS_LF, 1);
    }

    let result = ((read[0] as u16) << 8) | read[1] as u16;
    let result = result & 0x1fff;
    let result = result / 2;

    Ok(result)
}

pub fn read_ls() -> anyhow::Result<u16> {
    let write = [0xff, 0xff];
    let mut read = [0, 0];
    unsafe {
        let hw = HARDWARE.as_mut().unwrap();
        gpio_set_level(CS_LS, 0);
        hw.spi.transfer(&mut read, &write)?;
        gpio_set_level(CS_LS, 1);
    }

    let result = ((read[0] as u16) << 8) | read[1] as u16;
    let result = result & 0x1fff;
    let result = result / 2;

    Ok(result)
}

pub fn read_rf() -> anyhow::Result<u16> {
    let write = [0xff, 0xff];
    let mut read = [0, 0];
    unsafe {
        let hw = HARDWARE.as_mut().unwrap();
        gpio_set_level(CS_RF, 0);
        hw.spi.transfer(&mut read, &write)?;
        gpio_set_level(CS_RF, 1);
    }

    let result = ((read[0] as u16) << 8) | read[1] as u16;
    let result = result & 0x1fff;
    let result = result / 2;

    Ok(result)
}

pub fn read_rs() -> anyhow::Result<u16> {
    let write = [0xff, 0xff];
    let mut read = [0, 0];
    unsafe {
        let hw = HARDWARE.as_mut().unwrap();
        gpio_set_level(CS_RS, 0);
        hw.spi.transfer(&mut read, &write)?;
        gpio_set_level(CS_RS, 1);
    }

    let result = ((read[0] as u16) << 8) | read[1] as u16;
    let result = result & 0x1fff;
    let result = result / 2;

    Ok(result)
}

pub fn on_lf() -> anyhow::Result<()> {
    unsafe {
        gpio_set_level(EN_LF, 1);
    }
    Ok(())
}

pub fn off_lf() -> anyhow::Result<()> {
    unsafe {
        gpio_set_level(EN_LF, 0);
    }
    Ok(())
}

pub fn on_ls() -> anyhow::Result<()> {
    unsafe {
        gpio_set_level(EN_LS, 1);
    }
    Ok(())
}

pub fn off_ls() -> anyhow::Result<()> {
    unsafe {
        gpio_set_level(EN_LS, 0);
    }
    Ok(())
}

pub fn on_rf() -> anyhow::Result<()> {
    unsafe {
        gpio_set_level(EN_RF, 1);
    }
    Ok(())
}

pub fn off_rf() -> anyhow::Result<()> {
    unsafe {
        gpio_set_level(EN_RF, 0);
    }
    Ok(())
}

pub fn on_rs() -> anyhow::Result<()> {
    unsafe {
        gpio_set_level(EN_RS, 1);
    }
    Ok(())
}

pub fn off_rs() -> anyhow::Result<()> {
    unsafe {
        gpio_set_level(EN_RS, 0);
    }
    Ok(())
}
