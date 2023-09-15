#[derive(Debug, Default)]
pub struct Control {}

#[derive(Debug, Default, Clone, Copy)]
pub struct ControlContext {
    pub gyro_yaw: f32, // deg/sec
    pub ls: f32,       // %
    pub lf: f32,       // %
    pub rf: f32,       // %
    pub rs: f32,       // %
    pub enc_l: f32,    // m/sec
    pub enc_r: f32,    // m/sec
    pub batt: f32,     // V
}

impl Control {
    pub fn control(&self, context: &ControlContext) -> (bool, f32, f32) {
        (false, 0.0, 0.0)
    }
}
