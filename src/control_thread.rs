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
    Run,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlThreadCommand {
    None,
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
    loop {
        set_status(ControlThreadStatus::Idle);
        while (unsafe { COMMAND } == ControlThreadCommand::None) {
            FreeRtos::delay_ms(config::CONTROL_CYCLE);
            control::estimate_mouse_state();
        }

        let command = unsafe { COMMAND };
        set_command(ControlThreadCommand::None);

        match command {
            ControlThreadCommand::Run => {}
            _ => {}
        }

        control::control();
        FreeRtos::delay_ms(config::CONTROL_CYCLE);
    }
}

fn run(control: &mut control::ControlContext) {
    loop {
        motor::set_l(0.0);
        motor::set_r(0.0);
        motor::enable(false);
        FreeRtos::delay_ms(config::CONTROL_CYCLE);
    }
}
