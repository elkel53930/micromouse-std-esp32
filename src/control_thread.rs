use crate::config;
use crate::context;
use crate::control;
use crate::imu;
use crate::motor;
use esp_idf_hal::delay::FreeRtos;

use crate::CS;

pub fn control_thread() {
    let control = control::Control::default();
    let mut gyro = imu::GyroSensor::new();
    loop {
        /*
            gyro.measure_offset(|| {
                let _guard = CS.enter();
                context::get().gyro_yaw_raw
            });
        */
        context::ope(|ctx| {
            let _guard = CS.enter();
            let gyro_yaw = gyro.correct(ctx.gyro_yaw_raw);
            ctx.control_context.gyro_yaw = gyro_yaw;
        });

        let control_context: control::ControlContext = {
            let _guard = CS.enter();
            context::get().control_context.clone()
        };
        let (en, l, r) = control.control(&control_context);
        motor::set_l(l);
        motor::set_r(r);
        motor::enable(en);
        FreeRtos::delay_ms(config::CONTROL_CYCLE);
    }
}
