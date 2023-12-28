use crate::led::*;
use esp_idf_hal::delay::FreeRtos;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;

pub type Command = (LedColor, Option<&'static str>);

struct LedPattern {
    red_step: usize,
    green_step: usize,
    blue_step: usize,
    red_pattern: Option<&'static str>,
    green_pattern: Option<&'static str>,
    blue_pattern: Option<&'static str>,
}

pub fn init() -> anyhow::Result<Sender<Command>> {
    let (tx, rx): (Sender<Command>, Receiver<Command>) = mpsc::channel();

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
            // Receive all commands in the queue
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
                        // No more commands
                        break;
                    }
                };
            }

            // Update the LEDs
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

// Process LED pattern
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
