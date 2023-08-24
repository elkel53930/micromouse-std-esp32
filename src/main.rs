use esp_idf_hal::adc::{config::Config, AdcChannelDriver, AdcDriver, Atten11dB};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::timer;
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_sys::{
    mcpwm_config_t, mcpwm_counter_type_t_MCPWM_UP_COUNTER, mcpwm_duty_type_t_MCPWM_DUTY_MODE_0,
    mcpwm_generator_t_MCPWM_GEN_A, mcpwm_generator_t_MCPWM_GEN_B, mcpwm_gpio_init, mcpwm_init,
    mcpwm_io_signals_t_MCPWM0A, mcpwm_io_signals_t_MCPWM0B, mcpwm_set_duty,
    mcpwm_timer_t_MCPWM_TIMER_0, mcpwm_unit_t_MCPWM_UNIT_0,
};

static mut LED_GREEN: Option<PinDriver<'_, Gpio19, Output>> = None;
static mut LED_BLUE: Option<PinDriver<'_, Gpio20, Output>> = None;
static mut LED_RED: Option<PinDriver<'_, Gpio21, Output>> = None;

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

fn init_led(peripherals: &mut Peripherals) -> anyhow::Result<()> {
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
    Ok(())
}

fn main() -> anyhow::Result<()> {

    esp_idf_sys::link_patches();

    let mut peripherals = Peripherals::take().unwrap();

    // Initialize LEDs
    init_led(&mut peripherals)?;

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

    let mut mot_sleep = PinDriver::output(peripherals.pins.gpio38)?;
    mot_sleep.set_high()?;
    set_motor_speed(0.0);

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

    loop {
        let adc_value = adc.read(&mut adc_pin).unwrap();
        println!("ADC value: {}", adc_value);
        FreeRtos::delay_ms(100);
    }
}

fn timer_isr() {
    unsafe {
        LED_GREEN.as_mut().unwrap().toggle().unwrap();
        LED_BLUE.as_mut().unwrap().toggle().unwrap();
        LED_RED.as_mut().unwrap().toggle().unwrap();
    }
}
