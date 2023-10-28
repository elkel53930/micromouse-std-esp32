use crate::context;
use esp_idf_hal::adc::{config::Config, AdcChannelDriver, AdcDriver, Atten11dB, ADC1};
use esp_idf_hal::gpio::{
    Gpio1, Gpio14, Gpio15, Gpio16, Gpio2, Gpio3, Gpio4, Gpio5, Output, PinDriver,
};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::task::CriticalSection;

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

#[derive(Debug, Default, Clone, Copy)]
pub struct RawValue {
    ls_on: u16,
    ls_off: u16,
    lf_on: u16,
    lf_off: u16,
    rf_on: u16,
    rf_off: u16,
    rs_on: u16,
    rs_off: u16,
    batt: u16,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Context {
    enable: bool,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct PhysicalValue {
    ls: f32,
    lf: f32,
    rf: f32,
    rs: f32,
    batt: f32,
}

impl PhysicalValue {
    pub fn get_ls(&self) -> f32 {
        self.ls
    }
    pub fn get_lf(&self) -> f32 {
        self.lf
    }
    pub fn get_rf(&self) -> f32 {
        self.rf
    }
    pub fn get_rs(&self) -> f32 {
        self.rs
    }
    pub fn get_batt(&self) -> f32 {
        self.batt
    }
}

pub static mut RAW: context::WriteByInterrupt<RawValue> =
    context::WriteByInterrupt::Data(RawValue {
        ls_on: 0,
        ls_off: 0,
        lf_on: 0,
        lf_off: 0,
        rf_on: 0,
        rf_off: 0,
        rs_on: 0,
        rs_off: 0,
        batt: 0,
    });
pub static mut PHYSICAL: context::ShareWithThread<PhysicalValue> =
    context::ShareWithThread::Data(PhysicalValue {
        ls: 0.0,
        lf: 0.0,
        rf: 0.0,
        rs: 0.0,
        batt: 0.0,
    });
static mut HARDWARE: Option<WallSensorHardware<'static>> = None;
static mut CONTEXT: Option<Context> = None;
static CS: CriticalSection = CriticalSection::new();

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

        CONTEXT = Some(Context { enable: false });
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

pub fn physical_conversion() {
    let raw = unsafe { RAW.get() };
    let physical = PhysicalValue {
        ls: raw.ls_on as f32 - raw.ls_off as f32,
        lf: raw.lf_on as f32 - raw.lf_off as f32,
        rf: raw.rf_on as f32 - raw.rf_off as f32,
        rs: raw.rs_on as f32 - raw.rs_off as f32,
        batt: raw.batt as f32,
    };
    unsafe {
        let guard = CS.enter();
        PHYSICAL.access(&guard, |data| *data = physical);
    }
}

pub fn get_physical_value() -> PhysicalValue {
    unsafe {
        let mut physical_value = PhysicalValue {
            ls: 0.0,
            lf: 0.0,
            rf: 0.0,
            rs: 0.0,
            batt: 0.0,
        };
        PHYSICAL.access(&CS.enter(), |data| {
            physical_value = *data;
        });
        physical_value
    }
}
