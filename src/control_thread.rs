use crate::config;
use crate::control;
use crate::imu;
use crate::motor;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::task::CriticalSection;

static CS: CriticalSection = CriticalSection::new();

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
    let mut control = control::Control::default();

    loop {
        set_status(ControlThreadStatus::Idle);
        while (unsafe { COMMAND } == ControlThreadCommand::None) {
            FreeRtos::delay_ms(config::CONTROL_CYCLE);
        }

        match unsafe { COMMAND } {
            ControlThreadCommand::None => {}
            ControlThreadCommand::MeasureGyroOffset => {
                set_status(ControlThreadStatus::MeasureGyroOffset);
                measure_gyro_offset();
            }
            ControlThreadCommand::Run => {
                set_status(ControlThreadStatus::Run);
                run(&mut control);
            }
        }
    }
}

fn run(control: &mut control::Control) {
    loop {
        motor::set_l(0.0);
        motor::set_r(0.0);
        motor::enable(false);
        FreeRtos::delay_ms(config::CONTROL_CYCLE);
    }
}

fn measure_gyro_offset() {
    imu::measure_offset();
}
