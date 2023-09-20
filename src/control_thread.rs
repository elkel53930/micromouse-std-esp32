use crate::config;
use crate::context;
use crate::control;
use crate::imu;
use crate::motor;
use esp_idf_hal::delay::FreeRtos;

pub static CS: esp_idf_hal::task::CriticalSection = esp_idf_hal::task::CriticalSection::new();

static mut COMMAND: ControlThreadCommand = ControlThreadCommand::None;
static mut STATUS: ControlThreadStatus = ControlThreadStatus::Idle;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlThreadStatus {
    Idle,
    MeasureGyroOffset,
    Run,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlThreadCommand {
    None,
    MeasureGyroOffset,
    Run,
}

pub fn set_command(command: ControlThreadCommand) {
    unsafe {
        let _guard = CS.enter();
        COMMAND = command;
    }
}

pub fn set_status(status: ControlThreadStatus) {
    unsafe {
        let _guard = CS.enter();
        STATUS = status;
    }
}

pub fn get_status() -> ControlThreadStatus {
    unsafe { STATUS }
}

pub fn wait_idle(timeout: Option<u32>) -> anyhow::Result<()> {
    let interval = 100;
    let mut elapsed = 0;
    loop {
        if get_status() == ControlThreadStatus::Idle {
            return Ok(());
        }
        FreeRtos::delay_ms(interval);
        elapsed += interval;
        if let Some(timeout) = timeout {
            if elapsed > timeout {
                return Err(anyhow::anyhow!("timeout"));
            }
        }
    }

}

pub fn control_thread() -> ! {
    let control = control::Control::default();
    let mut gyro = imu::GyroSensor::new();

    loop {
        set_status(ControlThreadStatus::Idle);
        while (unsafe { COMMAND } == ControlThreadCommand::None) {
            FreeRtos::delay_ms(config::CONTROL_CYCLE);
        }

        match unsafe { COMMAND } {
            ControlThreadCommand::None => {}
            ControlThreadCommand::MeasureGyroOffset => {
                set_status(ControlThreadStatus::MeasureGyroOffset);
                measure_gyro_offset(&mut gyro);
            }
            ControlThreadCommand::Run => {
                set_status(ControlThreadStatus::Run);
                run(&mut gyro, &control);
            }
        }
    }
}

fn run(gyro: &mut imu::GyroSensor, control: &control::Control) {
    loop {
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

fn measure_gyro_offset(gyro: &mut imu::GyroSensor) {
    gyro.measure_offset(|| {
        let _guard = CS.enter();
        context::get().gyro_yaw_raw
    });
}
