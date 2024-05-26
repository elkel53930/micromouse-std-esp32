mod motor_control;
use crate::encoder;
use crate::imu;
use crate::led::{self, LedColor::*};
use crate::log_thread;
use crate::log_thread::LOG_LEN;
use crate::misc::correct_value;
use crate::ods;
use crate::ods::MicromouseState;
use crate::pid;
use crate::timer_interrupt::{sync_ms, wait_us};
use crate::wall_sensor;
use embedded_svc::utils::asyncify::timer;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::prelude::*;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;

#[derive(Debug, Serialize, Deserialize, Default)]
struct WsConfig {
    led_rise_time: u32,

    rs_reference: u16,
    ls_reference: u16,

    rs_threshold: u16,
    rf_threshold: u16,
    lf_threshold: u16,
    ls_threshold: u16,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct GyroConfig {
    correction_table: Vec<(i16, f32)>,
    correction_coefficient: f32,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct EncoderConfig {
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
    enc_cfg: EncoderConfig,
    battery_cfg: BatteryConfig,

    search_ctrl_cfg: SearchControlConfig,
}

#[derive(Debug, Serialize, Deserialize, Default)]
struct SearchControlConfig {
    ff_coeff_l: f32,
    ff_offset_l: f32,
    ff_coeff_r: f32,
    ff_offset_r: f32,
    vel_fwd: f32,
    v_pid: pid::PidParameter,
}

struct LogInfo {
    interval: u8,
    counter: u8,
    is_full: bool,
}

impl LogInfo {
    fn new() -> Self {
        Self {
            interval: 0,
            counter: 0,
            is_full: false,
        }
    }
}

struct ControlContext {
    ods: Arc<ods::Ods>,

    #[allow(unused)]
    log_tx: Sender<log_thread::LogCommand>,
    log_info: LogInfo,

    response_tx: Sender<Response>,
    command_rx: Receiver<Command>,

    config: ControlThreadConfig,

    ls_ena: bool,
    lf_ena: bool,
    rf_ena: bool,
    rs_ena: bool,
}

impl ControlContext {
    fn new(
        ods: Arc<ods::Ods>,
        log_tx: Sender<log_thread::LogCommand>,
        response_tx: Sender<Response>,
        command_rx: Receiver<Command>,
        config: ControlThreadConfig,
    ) -> Self {
        Self {
            ods,
            log_tx,
            log_info: LogInfo::new(),
            response_tx,
            command_rx,
            config,
            ls_ena: false,
            lf_ena: false,
            rf_ena: false,
            rs_ena: false,
        }
    }
    pub fn start_log(&mut self, interval: u8) {
        log::info!("Start logging.");
        self.log_info.counter = 0;
        self.log_info.is_full = false;
        self.ods.log.lock().unwrap().clear();
    }

    pub fn log(&mut self) {
        if self.log_info.is_full {
            return;
        }
        self.log_info.counter += 1;
        if self.log_info.counter >= self.log_info.interval {
            self.log_info.counter = 0;
            {
                let mut log = self.ods.log.lock().unwrap();
                log.push(*self.ods.micromouse.lock().unwrap());
            }
            if self.ods.log.lock().unwrap().len() >= LOG_LEN {
                log::warn!("Log is full.");
                self.log_info.is_full = true;
            }
        }
    }

    pub fn stop_log(&mut self) {
        let _ = self.log_tx.send(log_thread::LogCommand::Save);
    }
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Command {
    GyroCalibration,
    SetActivateWallSensor(bool, bool, bool, bool),
    SStart(f32),
    SForward,
    SStop,
    SRight,
    SLeft,
    SReturn,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Response {
    CalibrationDone(f32),
    CommandRequest,
}

fn measure(ctx: &mut ControlContext) -> anyhow::Result<()> {
    let ls_enable = ctx.ls_ena;
    let lf_enable = ctx.lf_ena;
    let rf_enable = ctx.rf_ena;
    let rs_enable = ctx.rs_ena;

    let batt = wall_sensor::read_batt()?;
    let batt_phy = correct_value(
        &ctx.config.battery_cfg.correction_table.as_slice(),
        batt as i16,
    );

    let (ls_raw, ls) = if ls_enable {
        let ls_off = wall_sensor::read_ls()?;
        wall_sensor::on_ls()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let ls_on = wall_sensor::read_ls()?;
        wall_sensor::off()?;
        let ls = ls_on - ls_off;
        (Some(ls), Some(ls > ctx.config.ws_cfg.ls_threshold))
    } else {
        (None, None)
    };

    let (lf_raw, lf) = if lf_enable {
        let lf_off = wall_sensor::read_lf()?;
        wall_sensor::on_lf()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let lf_on = wall_sensor::read_lf()?;
        wall_sensor::off()?;
        let lf = lf_on - lf_off;
        (Some(lf), Some(lf > ctx.config.ws_cfg.lf_threshold))
    } else {
        (None, None)
    };

    let (rf_raw, rf) = if rf_enable {
        let rf_off = wall_sensor::read_rf()?;
        wall_sensor::on_rf()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let rf_on = wall_sensor::read_rf()?;
        wall_sensor::off()?;
        let rf = rf_on - rf_off;
        (Some(rf), Some(rf > ctx.config.ws_cfg.rf_threshold))
    } else {
        (None, None)
    };

    let (rs_raw, rs) = if rs_enable {
        let rs_off = wall_sensor::read_rs()?;
        wall_sensor::on_rs()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let rs_on = wall_sensor::read_rs()?;
        wall_sensor::off()?;
        let rs = rs_on - rs_off;
        (Some(rs), Some(rs > ctx.config.ws_cfg.rs_threshold))
    } else {
        (None, None)
    };

    {
        let mut ws = ctx.ods.wall_sensor.lock().unwrap();
        ws.ls_raw = ls_raw;
        ws.lf_raw = lf_raw;
        ws.rf_raw = rf_raw;
        ws.rs_raw = rs_raw;
        ws.ls = ls;
        ws.lf = lf;
        ws.rf = rf;
        ws.rs = rs;
        ws.batt_raw = batt;
        ws.batt_phy = batt_phy;
    }

    wall_sensor::off()?;

    let gyro_x = imu::read()?;

    let encoder_l = encoder::read_l()?;
    let encoder_r = encoder::read_r()?;

    let gyro_x_phy = correct_value(&ctx.config.gyro_cfg.correction_table.as_slice(), gyro_x);
    let gyro_x_phy = gyro_x_phy * ctx.config.gyro_cfg.correction_coefficient;

    {
        let mut imu = ctx.ods.imu.lock().unwrap();
        imu.gyro_x_raw = gyro_x;
        imu.gyro_x_phy = gyro_x_phy - imu.gyro_x_offset;
    }

    {
        let mut enc = ctx.ods.encoder.lock().unwrap();
        enc.l_prev = enc.l;
        enc.r_prev = enc.r;
        enc.l = encoder_l;
        enc.r = encoder_r;

        // the value overflows at 16384
        let temp = enc.l as i32 - enc.l_prev as i32;
        enc.l_diff = if temp > 8192 {
            temp - 16384
        } else if temp < -8192 {
            temp + 16384
        } else {
            temp
        } as i16;
        enc.l_diff = -enc.l_diff;

        let temp = enc.r as i32 - enc.r_prev as i32;
        enc.r_diff = if temp > 8192 {
            temp - 16384
        } else if temp < -8192 {
            temp + 16384
        } else {
            temp
        } as i16;
    }

    Ok(())
}

fn reset_micromouse_state(ctx: &mut ControlContext) {
    let mut micromouse = ctx.ods.micromouse.lock().unwrap();
    *micromouse = MicromouseState::default();
}

fn update(ctx: &mut ControlContext) {
    let (enc_r_diff, enc_l_diff) = {
        let enc = ctx.ods.encoder.lock().unwrap();
        (enc.r_diff as f32, enc.l_diff as f32)
    };

    let v_r = enc_r_diff / ctx.config.enc_cfg.gear_ratio
        * ctx.config.enc_cfg.wheel_diameter
        * std::f32::consts::PI
        / 16384.0;
    let v_l = enc_l_diff / ctx.config.enc_cfg.gear_ratio
        * ctx.config.enc_cfg.wheel_diameter
        * std::f32::consts::PI
        / 16384.0;

    let velocity = (v_r + v_l) / 2.0;

    let gyro = {
        let imu = ctx.ods.imu.lock().unwrap();
        imu.gyro_x_phy
    };
    {
        let mut micromouse = ctx.ods.micromouse.lock().unwrap();
        micromouse.theta += gyro * 0.001;
        micromouse.v = velocity;
        micromouse.x += velocity * micromouse.theta.cos() * 0.001;
        micromouse.y += velocity * micromouse.theta.sin() * 0.001;
        micromouse.v_l = v_l;
        micromouse.v_r = v_r;
        micromouse.omega = gyro;
        micromouse.time = crate::timer_interrupt::get_ms();
    }
}

fn set_motor_duty(ctx: &ControlContext, duty_l: f32, duty_r: f32) {
    crate::motor::set_l(duty_l);
    crate::motor::set_r(duty_r);
    let mut micromouse = ctx.ods.micromouse.lock().unwrap();
    micromouse.duty_l = duty_l;
    micromouse.duty_r = duty_r;
}

fn gyro_calibration(ctx: &mut ControlContext) {
    // Measure gyro offset
    let mut gyro_offset = 0.0;
    led::on(Blue).unwrap();
    {
        let mut imu = ctx.ods.imu.lock().unwrap();
        imu.gyro_x_offset = 0.0;
    }

    for _ in 0..1000 {
        measure(ctx).unwrap();
        {
            let imu = ctx.ods.imu.lock().unwrap();
            gyro_offset += imu.gyro_x_phy as f32;
        }
        sync_ms();
    }

    {
        let mut imu = ctx.ods.imu.lock().unwrap();
        gyro_offset /= 1000.0;
        imu.gyro_x_offset = gyro_offset;
    }
    ctx.response_tx
        .send(Response::CalibrationDone(gyro_offset))
        .unwrap();
    led::off(Blue).unwrap();
}

pub fn init(
    ods: &Arc<ods::Ods>,
    log_tx: Sender<log_thread::LogCommand>,
) -> anyhow::Result<(Sender<Command>, Receiver<Response>)> {
    // Message queues
    let (tx_for_ope, rx): (Sender<Command>, Receiver<Command>) = mpsc::channel();
    let (tx, rx_for_ope): (Sender<Response>, Receiver<Response>) = mpsc::channel();

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
                Ok(cmd) => match cmd {
                    Command::GyroCalibration => {
                        gyro_calibration(&mut ctx);
                    }
                    Command::SetActivateWallSensor(ls, lf, rf, rs) => {
                        ctx.ls_ena = ls;
                        ctx.lf_ena = lf;
                        ctx.rf_ena = rf;
                        ctx.rs_ena = rs;
                    }
                    Command::SStart(distance) => {
                        motor_control::start(&mut ctx, distance).unwrap();
                    }
                    Command::SForward => {}
                    Command::SStop => {}
                    Command::SRight => {}
                    Command::SLeft => {}
                    Command::SReturn => {}
                },
                Err(mpsc::TryRecvError::Empty) => {}
                Err(mpsc::TryRecvError::Disconnected) => {
                    log::warn!("command_rx disconnected.");
                }
            }
            measure(&mut ctx)?;
            update(&mut ctx);
            sync_ms()
        }
    })?;

    Ok((tx_for_ope, rx_for_ope))
}
