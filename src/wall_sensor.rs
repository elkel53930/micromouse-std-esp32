use esp_idf_hal::adc::{config::Config, AdcChannelDriver, AdcDriver, Atten11dB, ADC1};
use esp_idf_hal::gpio::{
    Gpio1, Gpio14, Gpio15, Gpio16, Gpio2, Gpio3, Gpio4, Gpio5, Output, PinDriver,
};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;

static mut AD: Option<AdcDriver<ADC1>> = None;
static mut LS: Option<AdcChannelDriver<'_, Gpio1, Atten11dB<ADC1>>> = None;
static mut LF: Option<AdcChannelDriver<'_, Gpio2, Atten11dB<ADC1>>> = None;
static mut RF: Option<AdcChannelDriver<'_, Gpio3, Atten11dB<ADC1>>> = None;
static mut RS: Option<AdcChannelDriver<'_, Gpio4, Atten11dB<ADC1>>> = None;
static mut ENA: Option<PinDriver<'_, Gpio14, Output>> = None;
static mut SEL0: Option<PinDriver<'_, Gpio15, Output>> = None;
static mut SEL1: Option<PinDriver<'_, Gpio16, Output>> = None;

static mut BATT: Option<AdcChannelDriver<'_, Gpio5, Atten11dB<ADC1>>> = None;

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        AD = Some(AdcDriver::new(
            peripherals.adc1.clone_unchecked(),
            &Config::default(),
        )?);
        LS = Some(AdcChannelDriver::new(
            peripherals.pins.gpio1.clone_unchecked(),
        )?);
        LF = Some(AdcChannelDriver::new(
            peripherals.pins.gpio2.clone_unchecked(),
        )?);
        RF = Some(AdcChannelDriver::new(
            peripherals.pins.gpio3.clone_unchecked(),
        )?);
        RS = Some(AdcChannelDriver::new(
            peripherals.pins.gpio4.clone_unchecked(),
        )?);
        BATT = Some(AdcChannelDriver::new(
            peripherals.pins.gpio5.clone_unchecked(),
        )?);
        SEL0 = Some(PinDriver::output(
            peripherals.pins.gpio15.clone_unchecked(),
        )?);
        SEL1 = Some(PinDriver::output(
            peripherals.pins.gpio16.clone_unchecked(),
        )?);
        ENA = Some(PinDriver::output(
            peripherals.pins.gpio14.clone_unchecked(),
        )?);
    }
    Ok(())
}

pub fn read_ls() -> anyhow::Result<u16> {
    let mut result;
    unsafe {
        result = AD.as_mut().unwrap().read(&mut LS.as_mut().unwrap())?;
    }
    Ok(result)
}

pub fn read_lf() -> anyhow::Result<u16> {
    let mut result;
    unsafe {
        result = AD.as_mut().unwrap().read(&mut LF.as_mut().unwrap())?;
    }
    Ok(result)
}

pub fn read_rf() -> anyhow::Result<u16> {
    let mut result;
    unsafe {
        result = AD.as_mut().unwrap().read(&mut RF.as_mut().unwrap())?;
    }
    Ok(result)
}

pub fn read_rs() -> anyhow::Result<u16> {
    let mut result;
    unsafe {
        result = AD.as_mut().unwrap().read(&mut RS.as_mut().unwrap())?;
    }
    Ok(result)
}

pub fn read_batt() -> anyhow::Result<u16> {
    let mut result;
    unsafe {
        result = AD.as_mut().unwrap().read(&mut BATT.as_mut().unwrap())?;
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
        SEL0.as_mut().unwrap().set_high()?;
        SEL1.as_mut().unwrap().set_low()?;
        ENA.as_mut().unwrap().set_high()?;
    }
    Ok(())
}

pub fn on_lf() -> anyhow::Result<()> {
    unsafe {
        SEL0.as_mut().unwrap().set_high()?;
        SEL1.as_mut().unwrap().set_high()?;
        ENA.as_mut().unwrap().set_high()?;
    }
    Ok(())
}

pub fn on_rf() -> anyhow::Result<()> {
    unsafe {
        SEL0.as_mut().unwrap().set_low()?;
        SEL1.as_mut().unwrap().set_high()?;
        ENA.as_mut().unwrap().set_high()?;
    }
    Ok(())
}

pub fn on_rs() -> anyhow::Result<()> {
    unsafe {
        SEL0.as_mut().unwrap().set_low()?;
        SEL1.as_mut().unwrap().set_low()?;
        ENA.as_mut().unwrap().set_high()?;
    }
    Ok(())
}

pub fn off() -> anyhow::Result<()> {
    unsafe {
        ENA.as_mut().unwrap().set_low()?;
    }
    Ok(())
}
