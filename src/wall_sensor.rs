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

pub fn init(_peripherals: &mut Peripherals) -> anyhow::Result<()> {
    Ok(())
}

pub fn read_ls() -> anyhow::Result<u16> {
    Ok(0)
}

pub fn read_lf() -> anyhow::Result<u16> {
    Ok(0)
}

pub fn read_rf() -> anyhow::Result<u16> {
    Ok(0)
}

pub fn read_rs() -> anyhow::Result<u16> {
    Ok(0)
}

pub fn read_batt() -> anyhow::Result<u16> {
    Ok(0)
}

/*
    SEL0 SEL1   LED
      1    0   LS(D1)
      1    1   LF(D2)
      0    1   RF(D3)
      0    0   RS(D4)
*/
pub fn on_ls() -> anyhow::Result<()> {
    Ok(())
}

pub fn on_lf() -> anyhow::Result<()> {
    Ok(())
}

pub fn on_rf() -> anyhow::Result<()> {
    Ok(())
}

pub fn on_rs() -> anyhow::Result<()> {
    Ok(())
}

pub fn off() -> anyhow::Result<()> {
    Ok(())
}
