use esp_idf_hal::adc::{config::Config, AdcChannelDriver, AdcDriver, Atten11dB};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::timer;
use esp_idf_sys as _;

mod led;
mod motor;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let mut peripherals = Peripherals::take().unwrap();

    // Initialize LEDs
    led::init(&mut peripherals)?;
    motor::init(&mut peripherals)?;

    let mut mot_sleep = PinDriver::output(peripherals.pins.gpio38)?;
    mot_sleep.set_high()?;
    motor::set_r(0.0);

    // Initialize the ADC
    let mut adc = AdcDriver::new(peripherals.adc1, &Config::new().calibration(true))?;
    let mut adc_pin: AdcChannelDriver<'_, Gpio5, Atten11dB<_>> =
        AdcChannelDriver::new(peripherals.pins.gpio5)?;

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
        let adc_value = adc.read(&mut adc_pin).unwrap();
        println!("ADC value: {}", adc_value);
        println!("R");
        motor::set_r(-25.0);
        FreeRtos::delay_ms(1000);
        motor::set_r(0.0);
        println!("L");
        motor::set_l(-25.0);
        FreeRtos::delay_ms(1000);
        motor::set_l(0.0);
    }
}

fn timer_isr() {
    led::toggle(led::LedColor::Green).unwrap();
    led::toggle(led::LedColor::Blue).unwrap();
    led::toggle(led::LedColor::Red).unwrap();
}
