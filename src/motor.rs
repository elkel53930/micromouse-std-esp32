use esp_idf_hal::gpio::{Gpio38, Output, PinDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;

// For now, esp-idf-hal does not support the MCPWM peripheral.
// So, we need to use the esp-idf-sys crate directly.
use esp_idf_sys::{
    mcpwm_config_t, mcpwm_counter_type_t_MCPWM_UP_COUNTER, mcpwm_duty_type_t_MCPWM_DUTY_MODE_0,
    mcpwm_generator_t_MCPWM_GEN_A, mcpwm_generator_t_MCPWM_GEN_B, mcpwm_gpio_init, mcpwm_init,
    mcpwm_io_signals_t_MCPWM0A, mcpwm_io_signals_t_MCPWM0B, mcpwm_io_signals_t_MCPWM1A,
    mcpwm_io_signals_t_MCPWM1B, mcpwm_set_duty, mcpwm_timer_t_MCPWM_TIMER_0,
    mcpwm_timer_t_MCPWM_TIMER_1, mcpwm_unit_t_MCPWM_UNIT_0, mcpwm_unit_t_MCPWM_UNIT_1,
};

// SLEEP pin for motor drivers
// This pin is shared between the three motor drivers
static mut SLEEP: Option<PinDriver<'_, Gpio38, Output>> = None;

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        SLEEP = Some(PinDriver::output(
            peripherals.pins.gpio38.clone_unchecked(),
        )?);
        SLEEP.as_mut().unwrap().set_low()?;
    }

    unsafe {
        // L
        mcpwm_gpio_init(mcpwm_unit_t_MCPWM_UNIT_0, mcpwm_io_signals_t_MCPWM0A, 34);
        mcpwm_gpio_init(mcpwm_unit_t_MCPWM_UNIT_0, mcpwm_io_signals_t_MCPWM0B, 35);
        let mut config_r: mcpwm_config_t = mcpwm_config_t::default();
        config_r.frequency = 100_000;
        config_r.cmpr_a = 0.0;
        config_r.cmpr_b = 0.0;
        config_r.counter_mode = mcpwm_counter_type_t_MCPWM_UP_COUNTER;
        config_r.duty_mode = mcpwm_duty_type_t_MCPWM_DUTY_MODE_0;

        mcpwm_init(
            mcpwm_unit_t_MCPWM_UNIT_0,
            mcpwm_timer_t_MCPWM_TIMER_0,
            &config_r,
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

        // R
        mcpwm_gpio_init(mcpwm_unit_t_MCPWM_UNIT_1, mcpwm_io_signals_t_MCPWM1A, 36);
        mcpwm_gpio_init(mcpwm_unit_t_MCPWM_UNIT_1, mcpwm_io_signals_t_MCPWM1B, 37);
        let mut config_l: mcpwm_config_t = mcpwm_config_t::default();
        config_l.frequency = 100_000;
        config_l.cmpr_a = 0.0;
        config_l.cmpr_b = 0.0;
        config_l.counter_mode = mcpwm_counter_type_t_MCPWM_UP_COUNTER;
        config_l.duty_mode = mcpwm_duty_type_t_MCPWM_DUTY_MODE_0;

        mcpwm_init(
            mcpwm_unit_t_MCPWM_UNIT_1,
            mcpwm_timer_t_MCPWM_TIMER_1,
            &config_l,
        );

        mcpwm_set_duty(
            mcpwm_unit_t_MCPWM_UNIT_1,
            mcpwm_timer_t_MCPWM_TIMER_1,
            mcpwm_generator_t_MCPWM_GEN_A,
            0.0,
        );
        mcpwm_set_duty(
            mcpwm_unit_t_MCPWM_UNIT_1,
            mcpwm_timer_t_MCPWM_TIMER_1,
            mcpwm_generator_t_MCPWM_GEN_B,
            0.0,
        );
    }
    Ok(())
}

/*
    Set the speed of the motor.
    The speed is a value between -100.0 and 100.0.
    A negative value means reverse.
*/
pub fn set_l(speed: f32) {
    /*
        | IN1 | IN2 |   OUT   |
        +-----+-----+---------+
        |  0  |  0  |  Coast  |
        |  0  |  1  | Reverse |
        |  1  |  0  | Forward |
        |  1  |  1  |  Brake  |

        Since Device logic is as shown above,
        it is necessary to switch the port
        that outputs PWM for forward and reverse rotation.
    */
    if speed < 0.0 {
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
                100.0 + speed,
            );
        }
    } else {
        unsafe {
            mcpwm_set_duty(
                mcpwm_unit_t_MCPWM_UNIT_0,
                mcpwm_timer_t_MCPWM_TIMER_0,
                mcpwm_generator_t_MCPWM_GEN_A,
                100.0 - speed,
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

pub fn set_r(speed: f32) {
    if speed < 0.0 {
        unsafe {
            mcpwm_set_duty(
                mcpwm_unit_t_MCPWM_UNIT_1,
                mcpwm_timer_t_MCPWM_TIMER_1,
                mcpwm_generator_t_MCPWM_GEN_A,
                100.0,
            );
            mcpwm_set_duty(
                mcpwm_unit_t_MCPWM_UNIT_1,
                mcpwm_timer_t_MCPWM_TIMER_1,
                mcpwm_generator_t_MCPWM_GEN_B,
                100.0 + speed,
            );
        }
    } else {
        unsafe {
            mcpwm_set_duty(
                mcpwm_unit_t_MCPWM_UNIT_1,
                mcpwm_timer_t_MCPWM_TIMER_1,
                mcpwm_generator_t_MCPWM_GEN_A,
                100.0 - speed,
            );
            mcpwm_set_duty(
                mcpwm_unit_t_MCPWM_UNIT_1,
                mcpwm_timer_t_MCPWM_TIMER_1,
                mcpwm_generator_t_MCPWM_GEN_B,
                100.0,
            );
        }
    }
}

pub fn enable(en: bool) {
    unsafe {
        if en {
            SLEEP.as_mut().unwrap().set_high().unwrap();
        } else {
            SLEEP.as_mut().unwrap().set_low().unwrap();
        }
    }
}
