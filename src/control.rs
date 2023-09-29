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

pub fn estimate_mouse_state() {
    unsafe {
        let guard = CS.enter();
        CONTROL.access(&guard, |control| {
            control.angle_yaw +=
                imu::get_physical_value() * (config::CONTROL_CYCLE as f32 / 1000.0);
        });
    }
}

pub fn control() -> (bool, f32, f32) {
    estimate_mouse_state();
    (false, 0.0, 0.0)
}

pub fn get_angle_yaw() -> f32 {
    unsafe { CONTROL.get().angle_yaw }
}
