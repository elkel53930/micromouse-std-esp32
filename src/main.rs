use esp_idf_hal::adc::{config::Config, AdcChannelDriver, AdcDriver, Atten11dB};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_sys::{
    mcpwm_config_t, mcpwm_counter_type_t_MCPWM_UP_COUNTER, mcpwm_duty_type_t_MCPWM_DUTY_MODE_0,
    mcpwm_generator_t_MCPWM_GEN_A, mcpwm_generator_t_MCPWM_GEN_B, mcpwm_gpio_init, mcpwm_init,
    mcpwm_io_signals_t_MCPWM0A, mcpwm_io_signals_t_MCPWM0B, mcpwm_set_duty, mcpwm_set_pin,
    mcpwm_timer_t_MCPWM_TIMER_0, mcpwm_unit_t_MCPWM_UNIT_0,
};

fn set_motor_speed(speed: f32) {
    if speed > 0.0 {
        // 正転
        unsafe {
            mcpwm_set_duty(
                mcpwm_unit_t_MCPWM_UNIT_0,
                mcpwm_timer_t_MCPWM_TIMER_0,
                mcpwm_generator_t_MCPWM_GEN_A,
                100.0,
            );
            mcpwm_set_duty(
                mcpwm_unit_t_MCPWM_UNIT_0,
                mcpwm_timer_t_MCPWM_TIMER_0,
                mcpwm_generator_t_MCPWM_GEN_B,
                100.0 - speed,
            );
        }
    } else {
        // 逆転
        unsafe {
            mcpwm_set_duty(
                mcpwm_unit_t_MCPWM_UNIT_0,
                mcpwm_timer_t_MCPWM_TIMER_0,
                mcpwm_generator_t_MCPWM_GEN_A,
                100.0 + speed,
            );
            mcpwm_set_duty(
                mcpwm_unit_t_MCPWM_UNIT_0,
                mcpwm_timer_t_MCPWM_TIMER_0,
                mcpwm_generator_t_MCPWM_GEN_B,
                100.0,
            );
        }
    }
}

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();

    // Initialize the blue LED
    let mut green = PinDriver::output(peripherals.pins.gpio19)?;
    let mut blue = PinDriver::output(peripherals.pins.gpio20)?;
    let mut red = PinDriver::output(peripherals.pins.gpio21)?;
    //    let mut cwccw_r = PinDriver::output(peripherals.pins.gpio37)?; // CWCCW_R
    //    let _ = PinDriver::output(peripherals.pins.gpio36)?; // PWM_R
    let mut mot_sleep = PinDriver::output(peripherals.pins.gpio38)?;

    unsafe {
        //        mcpwm_gpio_init(mcpwm_unit_t_MCPWM_UNIT_0, mcpwm_io_signals_t_MCPWM0A, 36);
        //        mcpwm_gpio_init(mcpwm_unit_t_MCPWM_UNIT_0, mcpwm_io_signals_t_MCPWM0B, 37);
        mcpwm_gpio_init(mcpwm_unit_t_MCPWM_UNIT_0, mcpwm_io_signals_t_MCPWM0A, 34);
        mcpwm_gpio_init(mcpwm_unit_t_MCPWM_UNIT_0, mcpwm_io_signals_t_MCPWM0B, 35);
        let mut config: mcpwm_config_t = mcpwm_config_t::default();
        config.frequency = 10_000;
        config.cmpr_a = 0.0;
        config.cmpr_b = 0.0;
        config.counter_mode = mcpwm_counter_type_t_MCPWM_UP_COUNTER;
        config.duty_mode = mcpwm_duty_type_t_MCPWM_DUTY_MODE_0;

        mcpwm_init(
            mcpwm_unit_t_MCPWM_UNIT_0,
            mcpwm_timer_t_MCPWM_TIMER_0,
            &config,
        );

        mcpwm_set_duty(
            mcpwm_unit_t_MCPWM_UNIT_0,
            mcpwm_timer_t_MCPWM_TIMER_0,
            mcpwm_generator_t_MCPWM_GEN_A,
            0.0,
        );
        mcpwm_set_duty(
            mcpwm_unit_t_MCPWM_UNIT_0,
            mcpwm_timer_t_MCPWM_TIMER_0,
            mcpwm_generator_t_MCPWM_GEN_B,
            0.0,
        );
    }

    mot_sleep.set_high()?;

    let mut speed = 0.0;
    let mut diff = 4.0;

    // Initialize the ADC
    let mut adc = AdcDriver::new(peripherals.adc1, &Config::new().calibration(true))?;
    let mut adc_pin: AdcChannelDriver<'_, Gpio5, Atten11dB<_>> =
        AdcChannelDriver::new(peripherals.pins.gpio5)?;

    loop {
        blue.set_low()?;
        let adc_value = adc.read(&mut adc_pin).unwrap();
        blue.set_high()?;
        red.set_low()?;
        println!("ADC value: {}", adc_value);
        red.set_high()?;
        green.set_low()?;
        FreeRtos::delay_ms(50);
        green.set_high()?;
        FreeRtos::delay_ms(50);

        speed = speed + diff;
        if speed >= 50.0 {
            speed = 50.0;
            diff = -2.0;
        } else if speed <= -50.0 {
            speed = -50.0;
            diff = 2.0;
        }
        println!("speed: {}", speed);
        set_motor_speed(speed);
    }
}
