use esp_idf_hal::gpio::{Gpio17, Gpio19, Gpio21, Output, PinDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;

static mut LED_GREEN: Option<PinDriver<'_, Gpio17, Output>> = None;
static mut LED_BLUE: Option<PinDriver<'_, Gpio19, Output>> = None;
static mut LED_RED: Option<PinDriver<'_, Gpio21, Output>> = None;

#[derive(Debug)]
pub enum LedColor {
    Green,
    Blue,
    Red,
}

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        LED_GREEN = Some(PinDriver::output(
            peripherals.pins.gpio17.clone_unchecked(),
        )?);
        LED_BLUE = Some(PinDriver::output(
            peripherals.pins.gpio19.clone_unchecked(),
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

// Turn on LED
// To use from other than LED threads, set None to the LED pattern command.
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

// Turn off LED
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

// Toggle LED
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
