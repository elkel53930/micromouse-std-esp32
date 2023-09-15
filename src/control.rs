#[derive(Debug, Default)]
pub struct Control {}

#[derive(Debug, Default)]
pub struct ControlContext {
    gyro_yaw: f32, // deg/sec
    ls: f32,       // %
    lf: f32,       // %
    rf: f32,       // %
    rs: f32,       // %
    enc_l: f32,    // m/sec
    enc_r: f32,    // m/sec
    batt: f32,     // V
}

impl Control {
    pub fn control(&self, contex: &ControlContext) -> (bool, f32, f32) {
        (false, 0.0, 0.0)
    }
}
