use crate::config;
use crate::config::CONTROL_CYCLE;
use crate::context;
use crate::imu;
use esp_idf_hal::task::CriticalSection;

static CS: CriticalSection = CriticalSection::new();

pub struct ControlContext {
    angle_yaw: f32,
}

pub static mut CONTROL: context::ShareWithThread<ControlContext> =
    context::ShareWithThread::PreInit;

pub fn init() {
    unsafe {
        CONTROL = context::ShareWithThread::Data(ControlContext { angle_yaw: 0.0 });
    }
}

pub fn control() -> (bool, f32, f32) {
    (false, 0.0, 0.0)
}

pub fn get_angle_yaw() -> f32 {
    unsafe { CONTROL.get().angle_yaw }
}
