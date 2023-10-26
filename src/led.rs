use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{Gpio19, Gpio20, Gpio21, Output, PinDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;

static mut LED_GREEN: Option<PinDriver<'_, Gpio19, Output>> = None;
static mut LED_BLUE: Option<PinDriver<'_, Gpio20, Output>> = None;
static mut LED_RED: Option<PinDriver<'_, Gpio21, Output>> = None;
static mut PATTERN: LedPattern = LedPattern {
    red_step: 0,
    green_step: 0,
    blue_step: 0,
    red_pattern: None,
    green_pattern: None,
    blue_pattern: None,
};

#[derive(Debug)]
pub enum LedColor {
    Green,
    Blue,
    Red,
}

pub type LedCommand = (LedColor, Option<&'static str>);

struct LedPattern {
    red_step: usize,
    green_step: usize,
    blue_step: usize,
    red_pattern: Option<&'static str>,
    green_pattern: Option<&'static str>,
    blue_pattern: Option<&'static str>,
}

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<Sender<LedCommand>> {
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
    let (tx, rx): (Sender<LedCommand>, Receiver<LedCommand>) = mpsc::channel();

    thread::spawn(move || {
        let mut ctx: LedPattern = LedPattern {
            red_step: 0,
            green_step: 0,
            blue_step: 0,
            red_pattern: None,
            green_pattern: None,
            blue_pattern: None,
        };
        loop {
            loop {
                match rx.try_recv() {
                    Ok(cmd) => match cmd.0 {
                        LedColor::Green => {
                            ctx.green_pattern = cmd.1;
                            ctx.green_step = 0;
                        }
                        LedColor::Blue => {
                            ctx.blue_pattern = cmd.1;
                            ctx.blue_step = 0;
                        }
                        LedColor::Red => {
                            ctx.red_pattern = cmd.1;
                            ctx.red_step = 0;
                        }
                    },
                    _ => {
                        break;
                    }
                };
            }
            ctx.red_step = pattern_internal(ctx.red_pattern, ctx.red_step, LedColor::Red).unwrap();
            ctx.green_step =
                pattern_internal(ctx.green_pattern, ctx.green_step, LedColor::Green).unwrap();
            ctx.blue_step =
                pattern_internal(ctx.blue_pattern, ctx.blue_step, LedColor::Blue).unwrap();
            FreeRtos::delay_ms(100);
        }
    });

    Ok(tx)
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

fn pattern_internal(pattern: Option<&str>, step: usize, color: LedColor) -> anyhow::Result<usize> {
    if pattern.is_none() {
        return Ok(0);
    }
    let pattern = pattern.unwrap().as_bytes();
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
