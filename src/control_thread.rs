use crate::encoder;
use crate::imu;
use crate::led::{self, LedColor::*};
use crate::log_thread;
use crate::misc::correct_value;
use crate::ods;
use crate::ods::MicromouseState;
use crate::timer_interrupt::{sync_ms, wait_us};
use crate::wall_sensor;
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
}

struct ControlContext {
    ods: Arc<ods::Ods>,

    #[allow(unused)]
    log_tx: Sender<log_thread::LogCommand>,

    response_tx: Sender<Response>,
    command_rx: Receiver<Command>,

    config: ControlThreadConfig,

    ls_ena: bool,
    lf_ena: bool,
    rf_ena: bool,
    rs_ena: bool,
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

fn measure(
    ctx: &mut ControlContext,
    ls_enable: bool,
    lf_enable: bool,
    rf_enable: bool,
    rs_enable: bool,
) -> anyhow::Result<()> {
    let batt = wall_sensor::read_batt()?;
    let batt_phy = correct_value(
        &ctx.config.battery_cfg.correction_table.as_slice(),
        batt as i16,
    );

    let (ls_raw, ls) = if ls_enable {
        wall_sensor::on_ls()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let ls_on = wall_sensor::read_ls()?;

        wall_sensor::off()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let ls_off = wall_sensor::read_ls()?;

        let ls = ls_on - ls_off;
        (Some(ls), Some(ls > ctx.config.ws_cfg.ls_threshold))
    } else {
        (None, None)
    };

    let (lf_raw, lf) = if lf_enable {
        wall_sensor::on_lf()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let lf_on = wall_sensor::read_lf()?;

        wall_sensor::off()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let lf_off = wall_sensor::read_lf()?;

        let lf = lf_on - lf_off;
        (Some(lf), Some(lf > ctx.config.ws_cfg.lf_threshold))
    } else {
        (None, None)
    };

    let (rf_raw, rf) = if rf_enable {
        wall_sensor::on_rf()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let rf_on = wall_sensor::read_rf()?;

        wall_sensor::off()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let rf_off = wall_sensor::read_rf()?;

        let rf = rf_on - rf_off;
        (Some(rf), Some(rf > ctx.config.ws_cfg.rf_threshold))
    } else {
        (None, None)
    };

    let (rs_raw, rs) = if rs_enable {
        wall_sensor::on_rs()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let rs_on = wall_sensor::read_rs()?;

        wall_sensor::off()?;
        wait_us(ctx.config.ws_cfg.led_rise_time);
        let rs_off = wall_sensor::read_rs()?;

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
    let enc_diff = {
        let enc = ctx.ods.encoder.lock().unwrap();
        (enc.l_diff as f32 + enc.r_diff as f32) / 2.0
    };
    let velocity = enc_diff / ctx.config.enc_cfg.gear_ratio
        * ctx.config.enc_cfg.wheel_diameter
        * std::f32::consts::PI
        / 16384.0;

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
        micromouse.omega = gyro;
    }
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
        measure(ctx, false, false, false, false).unwrap();
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

    let config = read().unwrap_or(ControlThreadConfig::default());

    println!("{:?}", config);

    let mut ctx = ControlContext {
        ods: ods.clone(),
        response_tx: tx,
        log_tx: log_tx,
        command_rx: rx,
        config: config,
        ls_ena: false,
        lf_ena: false,
        rf_ena: false,
        rs_ena: false,
    };

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
            ctx.command_rx.try_recv().ok().map(|cmd| match cmd {
                Command::GyroCalibration => {
                    gyro_calibration(&mut ctx);
                }
                Command::SetActivateWallSensor(ls, lf, rf, rs) => {
                    ctx.ls_ena = ls;
                    ctx.lf_ena = lf;
                    ctx.rf_ena = rf;
                    ctx.rs_ena = rs;
                }
                Command::SStart(velocity) => {}
                Command::SForward => {}
                Command::SStop => {}
                Command::SRight => {}
                Command::SLeft => {}
                Command::SReturn => {}
            });
            measure(&mut ctx, true, true, true, true)?;
            // update(&mut ctx);
            sync_ms();
        }
    })?;

    Ok((tx_for_ope, rx_for_ope))
}
