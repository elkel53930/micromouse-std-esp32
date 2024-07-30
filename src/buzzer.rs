use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::ledc::*;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_sys::ledc_set_freq;

static mut DRIVER: Option<LedcDriver> = None;

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    let timer_driver = LedcTimerDriver::new(
        unsafe { peripherals.ledc.timer0.clone_unchecked() },
        &config::TimerConfig::new()
            .frequency(440.Hz().into())
            .resolution(Resolution::Bits8),
    )?;
    let mut driver = LedcDriver::new(
        unsafe { peripherals.ledc.channel0.clone_unchecked() },
        timer_driver,
        unsafe { peripherals.pins.gpio38.clone_unchecked() },
    )?;

    driver.set_duty(0)?;

    unsafe {
        DRIVER = Some(driver);
    }

    sound("gD")?;

    Ok(())
}

fn play(frequency: Hertz, duration_ms: i32) -> anyhow::Result<()> {
    unsafe { ledc_set_freq(0, 0, frequency.into()) };
    for _ in 0..duration_ms {
        unsafe {
            DRIVER
                .as_mut()
                .unwrap()
                .set_duty(DRIVER.as_mut().unwrap().get_max_duty() / 2)?;
        }
        FreeRtos::delay_ms(1);
    }
    unsafe {
        DRIVER.as_mut().unwrap().set_duty(0)?;
    }
    Ok(())
}
pub fn sound(melody: &str) -> anyhow::Result<()> {
    // 一文字ずつ取り出す
    for scale in melody.chars() {
        match scale {
            'c' => {
                play(524.Hz(), 150)?; // 262 * 2
            }
            'd' => {
                play(588.Hz(), 150)?; // 294 * 2
            }
            'e' => {
                play(660.Hz(), 150)?; // 330 * 2
            }
            'f' => {
                play(698.Hz(), 150)?; // 349 * 2
            }
            'g' => {
                play(784.Hz(), 150)?; // 392 * 2
            }
            'a' => {
                play(880.Hz(), 150)?; // 440 * 2
            }
            'b' => {
                play(988.Hz(), 150)?; // 494 * 2
            }
            'C' => {
                play(1048.Hz(), 150)?; // 262 * 4
            }
            'D' => {
                play(1176.Hz(), 150)?; // 294 * 4
            }
            'E' => {
                play(1320.Hz(), 150)?; // 330 * 4
            }
            'F' => {
                play(1396.Hz(), 150)?; // 349 * 4
            }
            'G' => {
                play(1568.Hz(), 150)?; // 392 * 4
            }
            'A' => {
                play(1760.Hz(), 150)?; // 440 * 4
            }
            'B' => {
                play(1976.Hz(), 150)?; // 494 * 4
            }
            _ => {
                FreeRtos::delay_ms(150);
            }
        }
    }
    Ok(())
}

/*
    Melody
      boot gD
      start b_b_DB_G
      goal D_DEEGG
      low_battt c_c_cc
      stuck bagf_e_ee
      fast goal DEDE_FG_AABB
      error b_decc
*/
