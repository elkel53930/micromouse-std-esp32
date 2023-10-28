use esp_idf_hal::adc::{config::Config, AdcChannelDriver, AdcDriver, Atten11dB, ADC1};
use esp_idf_hal::gpio::{
    Gpio1, Gpio14, Gpio15, Gpio16, Gpio2, Gpio3, Gpio4, Gpio5, Output, PinDriver,
};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;

struct WallSensorHardware<'a> {
    ad: AdcDriver<'a, ADC1>,
    ls: AdcChannelDriver<'a, Gpio1, Atten11dB<ADC1>>,
    lf: AdcChannelDriver<'a, Gpio2, Atten11dB<ADC1>>,
    rf: AdcChannelDriver<'a, Gpio3, Atten11dB<ADC1>>,
    rs: AdcChannelDriver<'a, Gpio4, Atten11dB<ADC1>>,
    ena: PinDriver<'a, Gpio14, Output>,
    sel0: PinDriver<'a, Gpio15, Output>,
    sel1: PinDriver<'a, Gpio16, Output>,
    batt: AdcChannelDriver<'a, Gpio5, Atten11dB<ADC1>>,
}

static mut HARDWARE: Option<WallSensorHardware<'static>> = None;

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        let ad = AdcDriver::new(peripherals.adc1.clone_unchecked(), &Config::default())?;
        let ls = AdcChannelDriver::new(peripherals.pins.gpio1.clone_unchecked())?;
        let lf = AdcChannelDriver::new(peripherals.pins.gpio2.clone_unchecked())?;
        let rf = AdcChannelDriver::new(peripherals.pins.gpio3.clone_unchecked())?;
        let rs = AdcChannelDriver::new(peripherals.pins.gpio4.clone_unchecked())?;
        let batt = AdcChannelDriver::new(peripherals.pins.gpio5.clone_unchecked())?;
        let sel0 = PinDriver::output(peripherals.pins.gpio15.clone_unchecked())?;
        let sel1 = PinDriver::output(peripherals.pins.gpio16.clone_unchecked())?;
        let ena = PinDriver::output(peripherals.pins.gpio14.clone_unchecked())?;
        HARDWARE = Some(WallSensorHardware {
            ad: ad,
            ls: ls,
            lf: lf,
            rf: rf,
            rs: rs,
            ena: ena,
            sel0: sel0,
            sel1: sel1,
            batt: batt,
        });
    }
    Ok(())
}

pub fn read_ls() -> anyhow::Result<u16> {
    #[allow(unused_mut)]
    let mut result;
    unsafe {
        result = HARDWARE
            .as_mut()
            .unwrap()
            .ad
            .read(&mut HARDWARE.as_mut().unwrap().ls)?;
    }
    Ok(result)
}

pub fn read_lf() -> anyhow::Result<u16> {
    #[allow(unused_mut)]
    let mut result;
    unsafe {
        result = HARDWARE
            .as_mut()
            .unwrap()
            .ad
            .read(&mut HARDWARE.as_mut().unwrap().lf)?;
    }
    Ok(result)
}

pub fn read_rf() -> anyhow::Result<u16> {
    #[allow(unused_mut)]
    let mut result;
    unsafe {
        result = HARDWARE
            .as_mut()
            .unwrap()
            .ad
            .read(&mut HARDWARE.as_mut().unwrap().rf)?;
    }
    Ok(result)
}

pub fn read_rs() -> anyhow::Result<u16> {
    #[allow(unused_mut)]
    let mut result;
    unsafe {
        result = HARDWARE
            .as_mut()
            .unwrap()
            .ad
            .read(&mut HARDWARE.as_mut().unwrap().rs)?;
    }
    Ok(result)
}

pub fn read_batt() -> anyhow::Result<u16> {
    #[allow(unused_mut)]
    let mut result;
    unsafe {
        result = HARDWARE
            .as_mut()
            .unwrap()
            .ad
            .read(&mut HARDWARE.as_mut().unwrap().batt)?;
    }
    Ok(result)
}

/*
    SEL0 SEL1   LED
      1    0   LS(D1)
      1    1   LF(D2)
      0    1   RF(D3)
      0    0   RS(D4)
*/
pub fn on_ls() -> anyhow::Result<()> {
    unsafe {
        HARDWARE.as_mut().unwrap().sel0.set_high()?;
        HARDWARE.as_mut().unwrap().sel1.set_low()?;
        HARDWARE.as_mut().unwrap().ena.set_high()?;
    }
    Ok(())
}

pub fn on_lf() -> anyhow::Result<()> {
    unsafe {
        HARDWARE.as_mut().unwrap().sel0.set_high()?;
        HARDWARE.as_mut().unwrap().sel1.set_high()?;
        HARDWARE.as_mut().unwrap().ena.set_high()?;
    }
    Ok(())
}

pub fn on_rf() -> anyhow::Result<()> {
    unsafe {
        HARDWARE.as_mut().unwrap().sel0.set_low()?;
        HARDWARE.as_mut().unwrap().sel1.set_high()?;
        HARDWARE.as_mut().unwrap().ena.set_high()?;
    }
    Ok(())
}

pub fn on_rs() -> anyhow::Result<()> {
    unsafe {
        HARDWARE.as_mut().unwrap().sel0.set_low()?;
        HARDWARE.as_mut().unwrap().sel1.set_low()?;
        HARDWARE.as_mut().unwrap().ena.set_high()?;
    }
    Ok(())
}

pub fn off() -> anyhow::Result<()> {
    unsafe {
        HARDWARE.as_mut().unwrap().ena.set_low()?;
    }
    Ok(())
}
