mod motor_control;
use crate::encoder;
use crate::imu;
use crate::led::{self, LedColor::*};
use crate::log_thread;
use crate::log_thread::LOG_LEN;
use crate::misc;
use crate::misc::correct_value;
use crate::mm_const;
use crate::ods;
use crate::ods::MicromouseState;
use crate::pid;
use crate::spin_mpsc::{self, SpinReceiver, SpinSender};
use crate::timer_interrupt::{sync_ms, wait_us};
use crate::wall_sensor;
use mm_maze::maze::Wall;
use motor_control::reset_controller;
use motor_control::turn_back;
use motor_control::turn_left;
use motor_control::turn_right;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::prelude::*;
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};

#[derive(Debug, Serialize, Deserialize, Default)]
struct WsConfig {
    led_rise_time: u32,

    rs_reference: u16,
    ls_reference: u16,

    rs_threshold: u16,
    rf_threshold: u16,
    lf_threshold: u16,
    ls_threshold: u16,

    wall_edge_enable: bool,
    wall_edge_position: f32,

    ls_correction_table: Vec<(u16, f32)>,
    rs_correction_table: Vec<(u16, f32)>,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct GyroConfig {
    correction_table: Vec<(i16, f32)>,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct MechanicalParameter {
    wheel_diameter: f32,
    gear_ratio: f32,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct BatteryConfig {
    correction_table: Vec<(i16, f32)>,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct SpeedConfig {
    velocity: f32,
    acceleration: f32,
    deceleration: f32,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct ControlThreadConfig {
    ws_cfg: WsConfig,
    gyro_cfg: GyroConfig,
    mech_param: MechanicalParameter,
    battery_cfg: BatteryConfig,

    search_ctrl_cfg: SearchControlConfig,

    judge_position: f32,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct SearchControlConfig {
    vel_fwd: f32,
    theta_pid: pid::PidParameter,
    omega_pid: pid::PidParameter,
    v_pid: pid::PidParameter,
    pos_pid: pid::PidParameter,
    wall_pid: pid::PidParameter,
}

struct LogInfo {
    interval: u8,
    counter: u8,
    is_full: bool,
    on_logging: bool,
}

impl LogInfo {
    fn new() -> Self {
        Self {
            interval: 0,
            counter: 0,
            is_full: false,
            on_logging: false,
        }
    }
}

#[derive(Debug, Serialize, Deserialize, PartialEq, Clone, Copy)]
enum WsStep {
    Side,
    Front,
}

#[derive(Debug, Serialize, Deserialize, PartialEq, Clone, Copy)]
enum TurnBackDirection {
    Left,
    Right,
}

impl TurnBackDirection {
    pub fn flip(&self) -> Self {
        match self {
            TurnBackDirection::Left => TurnBackDirection::Right,
            TurnBackDirection::Right => TurnBackDirection::Left,
        }
    }
}

struct ControlContext {
    ods: Arc<Mutex<ods::Ods>>,

    #[allow(unused)]
    log_tx: Sender<log_thread::LogCommand>,
    log_info: LogInfo,

    response_tx: SpinSender<Response>,
    command_rx: SpinReceiver<Command>,

    config: ControlThreadConfig,

    ws_ena: bool,
    ws_step: WsStep,

    ls_ref: u16,
    rs_ref: u16,

    theta_pid: pid::Pid,
    omega_pid: pid::Pid,
    v_pid: pid::Pid,
    pos_pid: pid::Pid,
    wall_pid: pid::Pid,

    v_ave: misc::MovingAverage,
    batt_ave: misc::MovingAverageInt,

    req_id: u16,

    previous_time: u32,

    position_reset_count: u16,

    turn_back_direction: TurnBackDirection,
}

impl ControlContext {
    fn new(
        ods: Arc<Mutex<ods::Ods>>,
        log_tx: Sender<log_thread::LogCommand>,
        response_tx: SpinSender<Response>,
        command_rx: SpinReceiver<Command>,
        config: ControlThreadConfig,
    ) -> Self {
        Self {
            ods,
            log_tx,
            log_info: LogInfo::new(),
            response_tx,
            command_rx,
            config,
            ws_ena: false,
            ws_step: WsStep::Side,
            ls_ref: 0,
            rs_ref: 0,
            theta_pid: pid::Pid::empty(),
            omega_pid: pid::Pid::empty(),
            v_pid: pid::Pid::empty(),
            pos_pid: pid::Pid::empty(),
            wall_pid: pid::Pid::empty(),
            v_ave: misc::MovingAverage::new(20),
            batt_ave: misc::MovingAverageInt::new(100),
            req_id: 0,
            previous_time: 0,
            position_reset_count: 0,
            turn_back_direction: TurnBackDirection::Left,
        }
    }
    pub fn start_log(&mut self, interval: u8) {
        self.log_info.counter = 0;
        self.log_info.is_full = false;
        self.log_info.interval = interval;
        self.log_info.on_logging = true;
        self.ods.lock().unwrap().log.clear();
    }

    pub fn log(&mut self) {
        if self.log_info.is_full || !self.log_info.on_logging {
            return;
        }
        self.log_info.counter += 1;
        if self.log_info.counter >= self.log_info.interval {
            let mut ods = self.ods.lock().unwrap();
            self.log_info.counter = 0;
            let micromouse = ods.micromouse.clone();
            ods.log.push(micromouse);
            if ods.log.len() >= LOG_LEN {
                self.log_info.is_full = true;
            }
        }
    }

    pub fn log_msg(&mut self, msg: String) {
        let mut ods = self.ods.lock().unwrap();
        let (min, sec, ms, us) = crate::timer_interrupt::get_time();
        let msg = format!("[{:02}:{:02}:{:03}:{:03}] {}", min, sec, ms, us, msg);
        ods.log_msg.push(msg);
        while ods.log_msg.len() >= log_thread::LOG_MSG_LEN {
            ods.log_msg.remove(0);
        }
    }

    pub fn stop_log(&mut self) {
        self.log_info.on_logging = false;
        let _ = self.log_tx.send(log_thread::LogCommand::Save);
    }

    pub fn set_ws_enable(&mut self, ena: bool) {
        self.ws_ena = ena;
    }

    pub fn reset_controllers(&mut self) {
        self.v_pid.reset();
        self.theta_pid.reset();
        self.omega_pid.reset();
        self.pos_pid.reset();
        self.wall_pid.reset();
    }

    pub fn request_command(&mut self) {
        self.log_msg(format!("Req cmd({})", self.req_id));
        self.response_tx.send(Response::CommandRequest(self.req_id));
        self.req_id += 1;
    }
}

#[derive(Debug, Serialize, Deserialize, PartialEq, Clone, Copy)]
pub enum Command {
    GyroCalibration,
    StartLog(u8), // The argument is the interval of logging
    StopLog,
    SetActivateWallSensor(bool),
    ResetController,
    SStart(f32),
    SForward,
    SStop,
    SRight,
    SLeft,
    SReturn,
    SPivot(f32), // The arguments are the angle
    Test,
}

impl Default for Command {
    fn default() -> Self {
        Command::Test
    }
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Response {
    CalibrationDone(f32),
    CommandRequest(u16),
}

fn measure(ctx: &mut ControlContext) -> anyhow::Result<()> {
    let batt = ctx.batt_ave.update(wall_sensor::read_batt()?.into()) as u16;
    let batt_phy = correct_value(
        &ctx.config.battery_cfg.correction_table.as_slice(),
        batt as i16,
    );

    if ctx.ws_ena {
        match ctx.ws_step {
            WsStep::Side => {
                let ls_off = wall_sensor::read_ls()?;
                wall_sensor::on_ls()?;
                wait_us(ctx.config.ws_cfg.led_rise_time);
                let ls_on = wall_sensor::read_ls()?;
                wall_sensor::off()?;
                let ls_raw = ls_on - ls_off;
                let ls_raw =
                    correct_value(ctx.config.ws_cfg.ls_correction_table.as_slice(), ls_raw) as u16;
                let ls = ls_raw > ctx.config.ws_cfg.ls_threshold;
                let ls = Some(Wall::from_bool(ls));

                let rs_off = wall_sensor::read_rs()?;
                wall_sensor::on_rs()?;
                wait_us(ctx.config.ws_cfg.led_rise_time);
                let rs_on = wall_sensor::read_rs()?;
                wall_sensor::off()?;
                let rs_raw = rs_on - rs_off;
                let rs_raw =
                    correct_value(ctx.config.ws_cfg.ls_correction_table.as_slice(), rs_raw) as u16;
                let rs = rs_raw > ctx.config.ws_cfg.rs_threshold;
                let rs = Some(Wall::from_bool(rs));

                {
                    let mut ods = ctx.ods.lock().unwrap();
                    ods.wall_sensor.ls_raw = Some(ls_raw);
                    ods.wall_sensor.rs_raw = Some(rs_raw);
                    ods.wall_sensor.ls = ls;
                    ods.wall_sensor.rs = rs;
                }
                ctx.ws_step = WsStep::Front;
            }
            WsStep::Front => {
                let lf_off = wall_sensor::read_lf()?;
                wall_sensor::on_lf()?;
                wait_us(ctx.config.ws_cfg.led_rise_time);
                let lf_on = wall_sensor::read_lf()?;
                wall_sensor::off()?;
                let lf_raw = lf_on - lf_off;
                let lf = lf_raw > ctx.config.ws_cfg.lf_threshold;
                let lf = Some(Wall::from_bool(lf));

                let rf_off = wall_sensor::read_rf()?;
                wall_sensor::on_rf()?;
                wait_us(ctx.config.ws_cfg.led_rise_time);
                let rf_on = wall_sensor::read_rf()?;
                wall_sensor::off()?;
                let rf_raw = rf_on - rf_off;
                let rf = rf_raw > ctx.config.ws_cfg.rf_threshold;
                let rf = Some(Wall::from_bool(rf));

                {
                    let mut ods = ctx.ods.lock().unwrap();
                    ods.wall_sensor.lf_raw = Some(lf_raw);
                    ods.wall_sensor.rf_raw = Some(rf_raw);
                    ods.wall_sensor.lf = lf;
                    ods.wall_sensor.rf = rf;
                }
                ctx.ws_step = WsStep::Side;
            }
        }
    } else {
        let mut ods = ctx.ods.lock().unwrap();
        ods.wall_sensor.batt_raw = batt;
        ods.wall_sensor.batt_phy = batt_phy;
        ods.wall_sensor.lf_raw = None;
        ods.wall_sensor.rf_raw = None;
        ods.wall_sensor.lf = None;
        ods.wall_sensor.rf = None;
        ods.wall_sensor.ls_raw = None;
        ods.wall_sensor.rs_raw = None;
        ods.wall_sensor.ls = None;
        ods.wall_sensor.rs = None;
        ctx.ws_step = WsStep::Side;
    }

    wall_sensor::off()?;

    let gyro_x = imu::read()?;

    let encoder_l = encoder::read_l()?;
    let encoder_r = encoder::read_r()?;

    let gyro_x_phy = correct_value(&ctx.config.gyro_cfg.correction_table.as_slice(), gyro_x);

    {
        let mut ods = ctx.ods.lock().unwrap();
        // IMU
        ods.imu.gyro_x_raw = gyro_x;
        ods.imu.gyro_x_phy = gyro_x_phy - ods.imu.gyro_x_offset;

        // Encoders
        ods.encoder.l_prev = ods.encoder.l;
        ods.encoder.r_prev = ods.encoder.r;
        ods.encoder.l = encoder_l;
        ods.encoder.r = encoder_r;
        // the value overflows at 16384
        let temp = ods.encoder.l as i32 - ods.encoder.l_prev as i32;
        ods.encoder.l_diff = if temp > 8192 {
            temp - 16384
        } else if temp < -8192 {
            temp + 16384
        } else {
            temp
        } as i16;
        ods.encoder.l_diff = -ods.encoder.l_diff;
        let temp = ods.encoder.r as i32 - ods.encoder.r_prev as i32;
        ods.encoder.r_diff = if temp > 8192 {
            temp - 16384
        } else if temp < -8192 {
            temp + 16384
        } else {
            temp
        } as i16;

        // Battery
        ods.wall_sensor.batt_raw = batt;
        ods.wall_sensor.batt_phy = batt_phy;
    }

    Ok(())
}

fn reset_micromouse_state(ctx: &mut ControlContext) {
    let mut ods = ctx.ods.lock().unwrap();
    ods.micromouse = MicromouseState::default();
}

fn update(ctx: &mut ControlContext) -> MicromouseState {
    let mut ods = ctx.ods.lock().unwrap();
    let current_time = crate::timer_interrupt::get_ms();
    let delta_step = if ctx.previous_time == 0 {
        1.0
    } else {
        (current_time - ctx.previous_time) as f32
    };

    ctx.previous_time = current_time;

    let dt = mm_const::DT as f32 * delta_step as f32;

    let enc_r_diff = ods.encoder.r_diff as f32;
    let enc_l_diff = ods.encoder.l_diff as f32;

    let v_r = enc_r_diff / ctx.config.mech_param.gear_ratio
        * ctx.config.mech_param.wheel_diameter
        * std::f32::consts::PI
        / 16384.0
        / delta_step;
    let v_l = enc_l_diff / ctx.config.mech_param.gear_ratio
        * ctx.config.mech_param.wheel_diameter
        * std::f32::consts::PI
        / 16384.0
        / delta_step;

    let velocity = (v_r + v_l) / 2.0;

    let gyro = ods.imu.gyro_x_phy;
    let ave_velo = ctx.v_ave.update(velocity); // moving average
    ods.micromouse.theta += gyro * dt;
    ods.micromouse.v = ave_velo;
    // Odometry is calculated by integrating the non-averaged velocity
    ods.micromouse.x += velocity * ods.micromouse.theta.cos() * dt;
    ods.micromouse.y += velocity * ods.micromouse.theta.sin() * dt;
    ods.micromouse.v_l = v_l;
    ods.micromouse.v_r = v_r;
    ods.micromouse.v_batt = ods.wall_sensor.batt_phy;
    ods.micromouse.omega = gyro;
    ods.micromouse.time = crate::timer_interrupt::get_ms();
    ods.micromouse.ls = ods.wall_sensor.ls_raw.unwrap_or(0);
    ods.micromouse.lf = ods.wall_sensor.lf_raw.unwrap_or(0);
    ods.micromouse.rf = ods.wall_sensor.rf_raw.unwrap_or(0);
    ods.micromouse.rs = ods.wall_sensor.rs_raw.unwrap_or(0);

    ods.micromouse.ls_wall = ods.wall_sensor.ls.unwrap_or(Wall::Absent);
    ods.micromouse.lf_wall = ods.wall_sensor.lf.unwrap_or(Wall::Absent);
    ods.micromouse.rf_wall = ods.wall_sensor.rf.unwrap_or(Wall::Absent);
    ods.micromouse.rs_wall = ods.wall_sensor.rs.unwrap_or(Wall::Absent);

    ods.micromouse.delta_step = delta_step;
    ods.micromouse.wall_error = 0;

    ods.micromouse.clone()
}

fn set_motor_duty(ctx: &ControlContext, duty_l: f32, duty_r: f32) {
    crate::motor::set_l(duty_l);
    crate::motor::set_r(duty_r);
    let mut ods = ctx.ods.lock().unwrap();
    ods.micromouse.duty_l = duty_l;
    ods.micromouse.duty_r = duty_r;
}

fn gyro_calibration(ctx: &mut ControlContext) {
    // Measure gyro offset
    let mut gyro_offset = 0.0;
    led::on(Blue).unwrap();
    ctx.ods.lock().unwrap().imu.gyro_x_offset = 0.0;

    for _ in 0..1000 {
        measure(ctx).unwrap();
        gyro_offset += ctx.ods.lock().unwrap().imu.gyro_x_phy as f32;
        sync_ms();
    }

    ctx.ods.lock().unwrap().imu.gyro_x_offset = gyro_offset / 1000.0;
    ctx.response_tx.send(Response::CalibrationDone(gyro_offset));
    led::off(Blue).unwrap();
}

pub fn init(
    ods: &Arc<Mutex<ods::Ods>>,
    log_tx: Sender<log_thread::LogCommand>,
) -> anyhow::Result<(
    SpinSender<Command>,
    SpinReceiver<Response>,
    anyhow::Result<()>,
)> {
    // Message queues
    let (tx_for_ope, rx): (SpinSender<Command>, SpinReceiver<Command>) = spin_mpsc::channel();
    let (tx, rx_for_ope): (SpinSender<Response>, SpinReceiver<Response>) = spin_mpsc::channel();
    let mut config_success = Ok(());

    fn read() -> anyhow::Result<ControlThreadConfig> {
        let mut f = File::open("/sf/ctrl_cfg.json")?;
        let mut contents = String::new();
        f.read_to_string(&mut contents)?;
        let result = serde_json::from_str(&contents)?;
        return Ok(result);
    }

    let config = match read() {
        Ok(c) => c,
        Err(e) => {
            println!("❌Failed to read config: {:?}", e);
            config_success = Err(e);
            ControlThreadConfig::default()
        }
    };

    println!("{:?}", config);

    let mut ctx = ControlContext::new(ods.clone(), log_tx, tx, rx, config);

    // Spawn the control thread
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: None,
        stack_size: 4096,
        priority: 15,
        inherit: false, // don't inherit this configuration across threads
        pin_to_core: Some(esp_idf_hal::cpu::Core::Core1),
    }
    .set()?;

    std::thread::Builder::new().spawn(move || -> anyhow::Result<()> {
        wall_sensor::off()?;
        loop {
            match ctx.command_rx.try_recv() {
                Some(cmd) => match cmd {
                    Command::GyroCalibration => {
                        gyro_calibration(&mut ctx);
                    }
                    Command::StartLog(interval) => {
                        ctx.log_msg("StartLog".to_string());
                        ctx.start_log(interval);
                        ctx.log_msg("StartLog done".to_string());
                        ctx.request_command();
                    }
                    Command::StopLog => {
                        ctx.log_msg("StopLog".to_string());
                        ctx.stop_log();
                        ctx.log_msg("StopLog done".to_string());
                        ctx.request_command();
                    }
                    Command::SetActivateWallSensor(ena) => {
                        ctx.set_ws_enable(ena);
                    }
                    Command::ResetController => {
                        ctx.log_msg("ResetController".to_string());
                        reset_controller(&mut ctx);
                        ctx.log_msg("ResetController done".to_string());
                        ctx.request_command();
                    }
                    Command::SStart(distance) => {
                        ctx.log_msg(format!("SStart({})", distance));
                        ctx.set_ws_enable(true);
                        motor_control::start(&mut ctx, distance).unwrap();
                        ctx.log_msg("SStart done".to_string());
                    }
                    Command::SForward => {
                        ctx.log_msg("SForward".to_string());
                        motor_control::forward(&mut ctx, mm_const::BLOCK_LENGTH).unwrap();
                        ctx.log_msg("SForward done".to_string());
                    }
                    Command::SStop => {
                        ctx.log_msg("SStop".to_string());
                        motor_control::stop(&mut ctx, mm_const::BLOCK_LENGTH / 2.0, true).unwrap();
                        ctx.log_msg("SStop done".to_string());
                    }
                    Command::SRight => {
                        ctx.log_msg("SRight".to_string());
                        turn_right(&mut ctx).unwrap();
                        ctx.log_msg("SRight done".to_string());
                    }
                    Command::SLeft => {
                        ctx.log_msg("SLeft".to_string());
                        turn_left(&mut ctx).unwrap();
                        ctx.log_msg("SLeft done".to_string());
                    }
                    Command::SReturn => {
                        ctx.log_msg("SReturn".to_string());
                        turn_back(&mut ctx).unwrap();
                        ctx.log_msg("SReturn done".to_string());
                    }
                    Command::SPivot(angle) => {
                        ctx.log_msg(format!("SPivot({})", angle));
                        motor_control::pivot(&mut ctx, angle, angle / std::f32::consts::PI / 2.0)
                            .unwrap();
                        ctx.log_msg("SPivot done".to_string());
                        ctx.request_command();
                    }
                    Command::Test => {
                        ctx.log_msg("Test".to_string());
                        motor_control::test(&mut ctx).unwrap();
                        ctx.log_msg("Test done".to_string());
                    }
                },
                None => {}
            }
            measure(&mut ctx)?;
            update(&mut ctx);
            sync_ms()
        }
    })?;

    Ok((tx_for_ope, rx_for_ope, config_success))
}
