use esp_idf_hal::gpio::{Gpio19, Gpio20, Gpio21, Output, PinDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
static mut LED_GREEN: Option<PinDriver<'_, Gpio19, Output>> = None;
static mut LED_BLUE: Option<PinDriver<'_, Gpio20, Output>> = None;
static mut LED_RED: Option<PinDriver<'_, Gpio21, Output>> = None;

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        LED_GREEN = Some(PinDriver::output(
            peripherals.pins.gpio19.clone_unchecked(),
        )?);
        LED_BLUE = Some(PinDriver::output(
            peripherals.pins.gpio20.clone_unchecked(),
        )?);
        LED_RED = Some(PinDriver::output(
            peripherals.pins.gpio21.clone_unchecked(),
        )?);
        LED_GREEN.as_mut().unwrap().set_high()?;
        LED_BLUE.as_mut().unwrap().set_high()?;
        LED_RED.as_mut().unwrap().set_high()?;
    }
    Ok(())
}

#[allow(dead_code)]
pub enum LedColor {
    Green,
    Blue,
    Red,
}

#[allow(dead_code)]
pub fn on(color: LedColor) -> anyhow::Result<()> {
    unsafe {
        match color {
            LedColor::Green => LED_GREEN.as_mut().unwrap().set_low()?,
            LedColor::Blue => LED_BLUE.as_mut().unwrap().set_low()?,
            LedColor::Red => LED_RED.as_mut().unwrap().set_low()?,
        }
    }
    Ok(())
}

#[allow(dead_code)]
pub fn off(color: LedColor) -> anyhow::Result<()> {
    unsafe {
        match color {
            LedColor::Green => LED_GREEN.as_mut().unwrap().set_high()?,
            LedColor::Blue => LED_BLUE.as_mut().unwrap().set_high()?,
            LedColor::Red => LED_RED.as_mut().unwrap().set_high()?,
        }
    }
    Ok(())
}

#[allow(dead_code)]
pub fn toggle(color: LedColor) -> anyhow::Result<()> {
    unsafe {
        match color {
            LedColor::Green => LED_GREEN.as_mut().unwrap().toggle()?,
            LedColor::Blue => LED_BLUE.as_mut().unwrap().toggle()?,
            LedColor::Red => LED_RED.as_mut().unwrap().toggle()?,
        }
    }
    Ok(())
}

#[derive(Default)]
pub struct LedPattern {
    red_step: usize,
    green_step: usize,
    blue_step: usize,
    make_10hz: usize,
    pub red_pattern: Option<&'static str>,
    pub green_pattern: Option<&'static str>,
    pub blue_pattern: Option<&'static str>,
}

impl LedPattern {
    pub fn new() -> Self {
        LedPattern {
            red_step: 0,
            green_step: 0,
            blue_step: 0,
            make_10hz: 0,
            red_pattern: None,
            green_pattern: None,
            blue_pattern: None,
        }
    }

    fn pattern_internal(
        &self,
        pattern: &str,
        step: usize,
        color: LedColor,
    ) -> anyhow::Result<usize> {
        let pattern = pattern.as_bytes();
        let on_char = b'1';
        let off_char = b'0';
        let mut step = step;

        if pattern.len() == 0 {
            return Ok(0);
        }

        if pattern.len() <= step {
            step = 0;
        }

        match pattern[step] {
            x if x == on_char => on(color)?,
            x if x == off_char => off(color)?,
            _ => {}
        }
        Ok(step + 1)
    }

    pub fn pattern(&mut self) -> anyhow::Result<()> {
        self.make_10hz = (self.make_10hz + 1) % 100;
        if self.make_10hz == 0 {
            self.red_step = self.pattern_internal(
                self.red_pattern.unwrap_or("0"),
                self.red_step,
                LedColor::Red,
            )?;
            self.green_step = self.pattern_internal(
                self.green_pattern.unwrap_or("0"),
                self.green_step,
                LedColor::Green,
            )?;
            self.blue_step = self.pattern_internal(
                self.blue_pattern.unwrap_or("0"),
                self.blue_step,
                LedColor::Blue,
            )?;
        }
        Ok(())
    }
}
