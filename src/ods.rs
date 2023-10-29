use std::sync::Mutex;

pub struct OdsImu {
    pub gyro_x_raw: i16,
    pub gyro_x_phy: f32,
    pub gyro_x_offset: f32,
}

pub struct OdsEncoder {
    pub l_raw: u16,
    pub r_raw: u16,
}

pub struct OdsWallSensor {
    pub ls_raw: Option<u16>,
    pub lf_raw: Option<u16>,
    pub rf_raw: Option<u16>,
    pub rs_raw: Option<u16>,
    pub batt_raw: u16,
}

pub struct Ods {
    pub imu: Mutex<OdsImu>,
    pub encoder: Mutex<OdsEncoder>,
    pub wall_sensor: Mutex<OdsWallSensor>,
}

impl Ods {
    pub fn new() -> Self {
        Ods {
            imu: Mutex::new(OdsImu {
                gyro_x_raw: 0,
                gyro_x_phy: 0.0,
                gyro_x_offset: 0.0,
            }),
            encoder: Mutex::new(OdsEncoder { l_raw: 0, r_raw: 0 }),
            wall_sensor: Mutex::new(OdsWallSensor {
                ls_raw: None,
                lf_raw: None,
                rf_raw: None,
                rs_raw: None,
                batt_raw: 0,
            }),
        }
    }
}
