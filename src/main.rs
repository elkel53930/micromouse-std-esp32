use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::timer;
use esp_idf_sys as _;
use std::thread;

#[macro_use]
pub mod uart;

mod console;
pub mod context;
pub mod control;
mod encoder;
mod fram_logger;
mod imu;
mod led;
mod motor;
mod wall_sensor;

#[allow(unused_imports)]
use led::LedColor::{Blue, Green, Red};

pub static CS: esp_idf_hal::task::CriticalSection = esp_idf_hal::task::CriticalSection::new();

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let mut peripherals = Peripherals::take().unwrap();

    // Initialize LEDs
    led::init(&mut peripherals)?;
    motor::init(&mut peripherals)?;
    wall_sensor::init(&mut peripherals)?;
    fram_logger::init(&mut peripherals)?;
    imu::init(&mut peripherals)?;
    encoder::init(&mut peripherals)?;

    // You can use println up to before uart:init.
    println!("init uart");
    FreeRtos::delay_ms(100);
    uart::init(&mut peripherals)?;
    // After uart:init, you can use uprintln.
    uprintln!("init uart done");

    let timer_config = timer::TimerConfig::new().auto_reload(true);
    let mut timer = timer::TimerDriver::new(peripherals.timer00, &timer_config)?;
    timer.set_alarm(100)?;
    unsafe {
        timer.subscribe(timer_isr)?;
    }

    timer.enable_alarm(true)?;
    timer.enable(true)?;

    {
        CS.enter(); // enter critical section
        context::ope(|ctx| {
            ctx.enable_ls = true;
            ctx.enable_lf = true;
            ctx.enable_rf = true;
            ctx.enable_rs = true;
        });
        led::set(Green, "1");
        led::set(Red, "10");
    } // end critical section

    thread::spawn(|| {
        let control = control::Control::default();
        let context: control::ControlContext = control::ControlContext::default();
        loop {
            let (en, l, r) = control.control(&context);
            motor::set_l(l);
            motor::set_r(r);
            motor::enable(en);
            FreeRtos::delay_ms(1);
        }
    });
    let _ = motor::enable(true);

    let mut console = console::Console::new();
    console.run();
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum InterruptSequence {
    ReadBattEnableLs,
    ReadLsEnableLf,
    ReadLfEnableRf,
    ReadRfEnableRs,
    ReadRsDisable,
    ReadImu,
    ReadEncoders,
    Led,
    Etc,
    None,
}

const SEQUENCE: [InterruptSequence; 10] = [
    InterruptSequence::ReadBattEnableLs,
    InterruptSequence::ReadLsEnableLf,
    InterruptSequence::ReadLfEnableRf,
    InterruptSequence::ReadRfEnableRs,
    InterruptSequence::ReadRsDisable,
    InterruptSequence::ReadImu,
    InterruptSequence::ReadEncoders,
    InterruptSequence::Led,
    InterruptSequence::Etc,
    InterruptSequence::None,
];

fn timer_isr() {
    interrupt().unwrap();
}

fn interrupt() -> anyhow::Result<()> {
    let mut ctx = context::get();
    let step = SEQUENCE[ctx.step as usize];
    ctx.step = (ctx.step + 1) % SEQUENCE.len() as u8;

    match step {
        InterruptSequence::ReadBattEnableLs => {
            ctx.batt = wall_sensor::read_batt()?;
            if ctx.enable_ls {
                wall_sensor::on_ls()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadLsEnableLf => {
            if ctx.enable_ls {
                ctx.ls = wall_sensor::read_ls()?;
            }

            if ctx.enable_lf {
                wall_sensor::on_lf()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadLfEnableRf => {
            if ctx.enable_lf {
                ctx.lf = wall_sensor::read_lf()?;
            }

            if ctx.enable_rf {
                wall_sensor::on_rf()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadRfEnableRs => {
            if ctx.enable_rf {
                ctx.rf = wall_sensor::read_rf()?;
            }

            if ctx.enable_rs {
                wall_sensor::on_rs()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadRsDisable => {
            if ctx.enable_rs {
                ctx.rs = wall_sensor::read_rs()?;
            }
            wall_sensor::off()?;
        }

        InterruptSequence::ReadImu => {
            ctx.gyro = imu::read()?;
        }

        InterruptSequence::ReadEncoders => {
            ctx.enc_l = encoder::read_l()?;
            ctx.enc_r = encoder::read_r()?;
        }

        InterruptSequence::Led => {
            led::pattern()?;
        }

        InterruptSequence::Etc => {}

        InterruptSequence::None => {}
    }
    context::set(&ctx);
    Ok(())
}
