use crate::log_thread;
use crate::mm_const;
use mm_maze::maze::{Maze, Wall};
use serde::{Deserialize, Serialize};

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
    pub ls: Option<Wall>,
    pub lf: Option<Wall>,
    pub rf: Option<Wall>,
    pub rs: Option<Wall>,

    // Battery voltage
    pub batt_raw: u16,
    pub batt_phy: f32,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub enum Event {
    None,
    CommandRequest,
    GoDone,
    Go,
    Pivot,
    Stop,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct MicromouseState {
    pub time: u32,   // Time [ms]
    pub x: f32,      // X coordinate [m]
    pub y: f32,      // Y coordinate [m]
    pub theta: f32,  // Heading [rad]
    pub omega: f32,  // Angular velocity [rad/s]
    pub v_batt: f32, // Battery voltage [V]
    pub v: f32,      // Velocity [m/s]
    pub v_l: f32,    // Left wheel velocity [m/s]
    pub v_r: f32,    // Right wheel velocity [m/s]
    pub duty_l: f32, // Left wheel duty [%]
    pub duty_r: f32, // Left wheel duty [%]
    pub ls: u16,     // Left side sensor value
    pub lf: u16,     // Left front sensor value
    pub rf: u16,     // Right front sensor value
    pub rs: u16,     // Right side sensor value
    pub ls_wall: Wall,
    pub lf_wall: Wall,
    pub rf_wall: Wall,
    pub rs_wall: Wall,
    pub v_integ: f32,     // Velocity integral
    pub pos_integ: f32,   // Position integral
    pub theta_integ: f32, // Heading integral
    pub omega_integ: f32, // Angular velocity integral
    pub target_x: f32,
    pub target_y: f32,
    pub target_v: f32,
    pub target_a: f32,
    pub target_theta: f32,
    pub target_omega: f32,
    pub ws_error: i16,
    pub event: Event,
    pub diff_pos: f32,
}

impl Default for MicromouseState {
    fn default() -> Self {
        MicromouseState {
            time: 0,
            x: mm_const::BLOCK_LENGTH / 2.0, // Start position
            y: mm_const::INITIAL_POSITION,
            theta: std::f32::consts::PI / 2.0, // Start orientation (North, y-axis positive)
            omega: 0.0,
            v_batt: 0.0,
            v: 0.0,
            v_l: 0.0,
            v_r: 0.0,
            duty_l: 0.0,
            duty_r: 0.0,
            ls: 0,
            lf: 0,
            rf: 0,
            rs: 0,
            ls_wall: Wall::Absent,
            lf_wall: Wall::Absent,
            rf_wall: Wall::Absent,
            rs_wall: Wall::Absent,
            v_integ: 0.0,
            pos_integ: 0.0,
            theta_integ: 0.0,
            omega_integ: 0.0,
            target_x: mm_const::BLOCK_LENGTH / 2.0,
            target_y: mm_const::INITIAL_POSITION,
            target_v: 0.0,
            target_a: 0.0,
            target_theta: std::f32::consts::PI / 2.0,
            target_omega: 0.0,
            ws_error: 0,
            event: Event::None,
            diff_pos: 0.0,
        }
    }
}

pub struct Ods {
    pub imu: OdsImu,
    pub encoder: OdsEncoder,
    pub wall_sensor: OdsWallSensor,
    pub micromouse: MicromouseState,
    pub log: Vec<MicromouseState>,
    pub maze: Maze,
}

impl Ods {
    pub fn new() -> Self {
        Ods {
            imu: OdsImu::default(),
            encoder: OdsEncoder::default(),
            wall_sensor: OdsWallSensor::default(),
            micromouse: MicromouseState::default(),
            log: Vec::with_capacity(log_thread::LOG_LEN),
            maze: Maze::new(mm_const::MAZE_WIDTH, mm_const::MAZE_HEIGHT),
        }
    }
}
