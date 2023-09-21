use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::task::CriticalSection;
use crate::config;

static CS: CriticalSection = CriticalSection::new();

pub fn control_thread() -> ! {
	let mut gyro = imu::GyroSensor::new();
	loop {
        {
            let cs = CS.enter();
            let gyro_yaw = gyro.correct(imu::get_value(&cs).unwrap());
            context::ope(&cs, |ctx| {
                ctx.control_context.gyro_yaw = gyro_yaw;
            });
        }
	}
	FreeRtos::delay_ms(config::CONTROL_CYCLE);
}