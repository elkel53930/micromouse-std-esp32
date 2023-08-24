use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::timer;
use esp_idf_sys as _;

mod led;
mod motor;
mod wall_sensor;

#[derive(Debug, Default)]
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

#[derive(Debug, Default)]
struct InterruptContext {
    step: u8,
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
        println!("{} {} {} {}", ls, lf, rf, rs);
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
    let step = unsafe {
        let step = INTERRUPT_CONTEXT.as_ref().unwrap().step;
        INTERRUPT_CONTEXT.as_mut().unwrap().step = (step + 1) % 10;
        step
    };
    let step = SEQUENCE[step as usize];

    match step {
        InterruptSequence::ReadBattEnableLs => {
            let batt = wall_sensor::read_batt();
            unsafe {
                SENSOR_DATA.as_mut().unwrap().batt = batt.unwrap();
            }
            let _ = wall_sensor::on_ls();
        }

        InterruptSequence::ReadLsEnableLf => {
            let ls = wall_sensor::read_ls();
            unsafe {
                SENSOR_DATA.as_mut().unwrap().ls = ls.unwrap();
            }
            let _ = wall_sensor::on_lf();
        }

        InterruptSequence::ReadLfEnableRf => {
            let lf = wall_sensor::read_lf();
            unsafe {
                SENSOR_DATA.as_mut().unwrap().lf = lf.unwrap();
            }
            let _ = wall_sensor::on_rf();
        }

        InterruptSequence::ReadRfEnableRs => {
            let rf = wall_sensor::read_rf();
            unsafe {
                SENSOR_DATA.as_mut().unwrap().rf = rf.unwrap();
            }
            let _ = wall_sensor::on_rs();
        }

        InterruptSequence::ReadRsDisable => {
            let rs = wall_sensor::read_rs();
            unsafe {
                SENSOR_DATA.as_mut().unwrap().rs = rs.unwrap();
            }
            let _ = wall_sensor::off();
        }

        InterruptSequence::ReadImu => {}

        InterruptSequence::ReadEncoders => {}

        InterruptSequence::Control => {}

        InterruptSequence::Noop => {}

        InterruptSequence::Etc => {}
    }
}
