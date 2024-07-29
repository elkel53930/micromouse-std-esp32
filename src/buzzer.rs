
use esp_idf_hal::ledc::*;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_sys::ledc_set_freq;

static mut DRIVER: Option<LedcDriver> = None;

#[derive(Debug, Clone, Copy)]
pub enum Scale {
    Do,
    Re,
    Mi,
    Fa,
    So,
    La,
    Si,
}

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    let timer_driver = LedcTimerDriver::new(
        unsafe { peripherals.ledc.timer0.clone_unchecked() },
        &config::TimerConfig::new().frequency(440.Hz().into()).resolution(Resolution::Bits8),
    )?;
    let mut driver = LedcDriver::new(
        unsafe { peripherals.ledc.channel0.clone_unchecked() },
        timer_driver,
        unsafe { peripherals.pins.gpio38.clone_unchecked() },
    )?;

    driver.set_duty(driver.get_max_duty() / 2)?;


    // driver.disable()?;

    unsafe {
        DRIVER = Some(driver);
    }

    Ok(())
}

pub fn on(scale : Scale) -> anyhow::Result<()> {
    let frequency = match scale {
        Scale::Do => 262.Hz(),
        Scale::Re => 294.Hz(),
        Scale::Mi => 330.Hz(),
        Scale::Fa => 349.Hz(),
        Scale::So => 392.Hz(),
        Scale::La => 440.Hz(),
        Scale::Si => 494.Hz(),
    };

    unsafe{
        ledc_set_freq(0, 0, frequency.into())
    };
    unsafe {
        DRIVER.as_mut().unwrap().set_duty(DRIVER.as_mut().unwrap().get_max_duty() / 2)?;
    }
    Ok(())
}

pub fn off() -> anyhow::Result<()> {
    unsafe {
        DRIVER.as_mut().unwrap().set_duty(0)?;
    }
    Ok(())
}
