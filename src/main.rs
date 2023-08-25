use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::timer;
use esp_idf_sys as _;

mod led;
mod motor;
mod wall_sensor;

#[derive(Debug, Default, Clone)]
struct SensorData {
    batt: u16,
    ls: u16,
    lf: u16,
    rf: u16,
    rs: u16,
    imu: u16,
    enc_l: u16,
    enc_r: u16,
}
static mut SENSOR_DATA: Option<SensorData> = None;

#[derive(Debug, Default, Clone)]
struct InterruptContext {
    step: u8,
    enable_ls: bool,
    enable_lf: bool,
    enable_rf: bool,
    enable_rs: bool,
}
static mut INTERRUPT_CONTEXT: Option<InterruptContext> = None;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    unsafe {
        SENSOR_DATA = Some(SensorData::default());
        INTERRUPT_CONTEXT = Some(InterruptContext::default());
    }

    let mut peripherals = Peripherals::take().unwrap();

    // Initialize LEDs
    led::init(&mut peripherals)?;
    motor::init(&mut peripherals)?;
    wall_sensor::init(&mut peripherals)?;

    let mut mot_sleep = PinDriver::output(peripherals.pins.gpio38)?;
    mot_sleep.set_high()?;
    motor::set_r(0.0);
    motor::set_l(0.0);

    let timer_config = timer::TimerConfig::new().auto_reload(true);
    let mut timer = timer::TimerDriver::new(peripherals.timer00, &timer_config)?;
    timer.set_alarm(100)?;
    unsafe {
        timer.subscribe(timer_isr)?;
    }

    timer.enable_alarm(true)?;
    timer.enable(true)?;

    motor::enable(true);

    loop {
        let ls = unsafe { SENSOR_DATA.as_ref().unwrap().ls };
        let lf = unsafe { SENSOR_DATA.as_ref().unwrap().lf };
        let rf = unsafe { SENSOR_DATA.as_ref().unwrap().rf };
        let rs = unsafe { SENSOR_DATA.as_ref().unwrap().rs };
        let batt = unsafe { SENSOR_DATA.as_ref().unwrap().batt };
        println!(
            "ls: {}, lf: {}, rf: {}, rs: {}, batt: {}",
            ls, lf, rf, rs, batt
        );
        FreeRtos::delay_ms(100);
    }
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
    Control,
    Noop,
    Etc,
}

const SEQUENCE: [InterruptSequence; 10] = [
    InterruptSequence::ReadBattEnableLs,
    InterruptSequence::ReadLsEnableLf,
    InterruptSequence::ReadLfEnableRf,
    InterruptSequence::ReadRfEnableRs,
    InterruptSequence::ReadRsDisable,
    InterruptSequence::ReadImu,
    InterruptSequence::ReadEncoders,
    InterruptSequence::Control,
    InterruptSequence::Noop,
    InterruptSequence::Etc,
];

fn timer_isr() {
    interrupt().unwrap();
}

fn interrupt() -> anyhow::Result<()> {
    let ctx = unsafe { INTERRUPT_CONTEXT.as_ref().unwrap().clone() };
    unsafe {
        let step = INTERRUPT_CONTEXT.as_ref().unwrap().step;
        INTERRUPT_CONTEXT.as_mut().unwrap().step = (step + 1) % 10;
    };
    let step = SEQUENCE[ctx.step as usize];

    match step {
        InterruptSequence::ReadBattEnableLs => {
            unsafe {
                SENSOR_DATA.as_mut().unwrap().batt = wall_sensor::read_batt()?;
            }
            if ctx.enable_ls {
                wall_sensor::on_ls()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadLsEnableLf => {
            if ctx.enable_ls {
                unsafe {
                    SENSOR_DATA.as_mut().unwrap().ls = wall_sensor::read_ls()?;
                }
            }

            if ctx.enable_lf {
                wall_sensor::on_lf()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadLfEnableRf => {
            if ctx.enable_lf {
                unsafe {
                    SENSOR_DATA.as_mut().unwrap().lf = wall_sensor::read_lf()?;
                }
            }

            if ctx.enable_rf {
                wall_sensor::on_rf()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadRfEnableRs => {
            if ctx.enable_rf {
                unsafe {
                    SENSOR_DATA.as_mut().unwrap().rf = wall_sensor::read_rf()?;
                }
            }

            if ctx.enable_rs {
                wall_sensor::on_rs()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadRsDisable => {
            if ctx.enable_rs {
                unsafe {
                    SENSOR_DATA.as_mut().unwrap().rs = wall_sensor::read_rs()?;
                }
            }
            wall_sensor::off()?;
        }

        InterruptSequence::ReadImu => {}

        InterruptSequence::ReadEncoders => {}

        InterruptSequence::Control => {}

        InterruptSequence::Noop => {}

        InterruptSequence::Etc => {}
    }
    Ok(())
}
