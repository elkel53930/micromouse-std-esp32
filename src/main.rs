use esp_idf_hal::adc::{config::Config, AdcChannelDriver, AdcDriver, Atten11dB};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();

    // Initialize the blue LED
    let mut green = PinDriver::output(peripherals.pins.gpio19)?;
    let mut blue = PinDriver::output(peripherals.pins.gpio20)?;
    let mut red = PinDriver::output(peripherals.pins.gpio21)?;

    // Initialize the ADC
    let mut adc = AdcDriver::new(peripherals.adc1, &Config::new().calibration(true))?;
    let mut adc_pin: AdcChannelDriver<'_, Gpio5, Atten11dB<_>> =
        AdcChannelDriver::new(peripherals.pins.gpio5)?;

    loop {
        blue.set_low()?;
        let adc_value = adc.read(&mut adc_pin).unwrap();
        blue.set_high()?;
        red.set_low()?;
        //        println!("ADC value: {}", adc_value);
        red.set_high()?;
        green.set_low()?;
        FreeRtos::delay_ms(50);
        green.set_high()?;
        FreeRtos::delay_ms(50);
    }
}
