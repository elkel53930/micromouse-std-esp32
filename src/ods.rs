use crate::log_thread;
use std::sync::Mutex;

#[derive(Debug, Default, Clone, Copy)]
pub struct OdsImu {
    pub gyro_x_raw: i16,
    pub gyro_x_phy: f32,
    pub gyro_x_offset: f32,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct OdsEncoder {
    pub l: u16,
    pub r: u16,
    pub l_prev: u16,
    pub r_prev: u16,
    pub l_diff: i16,
    pub r_diff: i16,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct OdsWallSensor {
    // Sensor values
    pub ls_raw: Option<u16>,
    pub lf_raw: Option<u16>,
    pub rf_raw: Option<u16>,
    pub rs_raw: Option<u16>,

    // Wall presence
    pub ls: Option<bool>,
    pub lf: Option<bool>,
    pub rf: Option<bool>,
    pub rs: Option<bool>,

    // Battery voltage
    pub batt_raw: u16,
    pub batt_phy: f32,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct MicromouseState {
    pub x: f32,     // X coordinate [m]
    pub y: f32,     // Y coordinate [m]
    pub theta: f32, // Heading [rad]
    pub omega: f32, // Angular velocity [rad/s]
    pub v: f32,     // Velocity [m/s]
}
pub struct Ods {
    pub imu: Mutex<OdsImu>,
    pub encoder: Mutex<OdsEncoder>,
    pub wall_sensor: Mutex<OdsWallSensor>,
    pub micromouse: Mutex<MicromouseState>,
    pub log: Mutex<Vec<log_thread::Log>>,
}

impl Ods {
    pub fn new() -> Self {
        Ods {
            imu: Mutex::new(OdsImu::default()),
            encoder: Mutex::new(OdsEncoder::default()),
            wall_sensor: Mutex::new(OdsWallSensor::default()),
            micromouse: Mutex::new(MicromouseState::default()),
            log: Mutex::new(Vec::with_capacity(log_thread::LOG_SIZE)),
        }
    }
}
