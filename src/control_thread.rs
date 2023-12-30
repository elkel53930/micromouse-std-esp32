use crate::config;
use crate::encoder;
use crate::imu;
use crate::led::{self, LedColor::*};
use crate::log_thread;
use crate::misc::{correct_value, FIR};
use crate::ods;
use crate::timer_interrupt::{sync_ms, wait_us};
use crate::wall_sensor;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;

struct WsConfig {
    led_rise_time: u32,

    rs_threshold: u16,
    rf_threshold: u16,
    lf_threshold: u16,
    ls_threshold: u16,
}

struct GyroConfig {
    fir: FIR<f32>,
    correction_table: Vec<(i16, f32)>,
    correction_coefficient: f32,
}

struct BatteryConfig {
    correction_table: Vec<(i16, f32)>,
}

struct ControlContext {
    ods: Arc<ods::Ods>,

    #[allow(unused)]
    log_tx: Sender<log_thread::LogCommand>,

    response_tx: Sender<Response>,
    command_rx: Receiver<Command>,
    ws_cfg: WsConfig,
    gyro_cfg: GyroConfig,
    battery_cfg: BatteryConfig,
}

#[derive(Debug, PartialEq, Clone, Copy)]
enum State {
    Idle(bool, bool, bool, bool),
    GyroCalibration,
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Command {
    GyroCalibration,
    SetActivateWallSensor(bool, bool, bool, bool),
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Response {
    Done,
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
    let batt_phy = correct_value(&ctx.battery_cfg.correction_table.as_slice(), batt as i16);

    let (ls_raw, ls) = if ls_enable {
        wall_sensor::on_ls()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let ls_on = wall_sensor::read_ls()?;

        wall_sensor::off()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let ls_off = wall_sensor::read_ls()?;

        let ls = ls_on - ls_off;
        (Some(ls), Some(ls > ctx.ws_cfg.ls_threshold))
    } else {
        (None, None)
    };

    let (lf_raw, lf) = if lf_enable {
        wall_sensor::on_lf()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let lf_on = wall_sensor::read_lf()?;

        wall_sensor::off()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let lf_off = wall_sensor::read_lf()?;

        let lf = lf_on - lf_off;
        (Some(lf), Some(lf > ctx.ws_cfg.lf_threshold))
    } else {
        (None, None)
    };

    let (rf_raw, rf) = if rf_enable {
        wall_sensor::on_rf()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let rf_on = wall_sensor::read_rf()?;

        wall_sensor::off()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let rf_off = wall_sensor::read_rf()?;

        let rf = rf_on - rf_off;
        (Some(rf), Some(rf > ctx.ws_cfg.rf_threshold))
    } else {
        (None, None)
    };

    let (rs_raw, rs) = if rs_enable {
        wall_sensor::on_rs()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let rs_on = wall_sensor::read_rs()?;

        wall_sensor::off()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let rs_off = wall_sensor::read_rs()?;

        let rs = rs_on - rs_off;
        (Some(rs), Some(rs > ctx.ws_cfg.rs_threshold))
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

    let gyro_x_phy = correct_value(&ctx.gyro_cfg.correction_table.as_slice(), gyro_x);
    let gyro_x_phy = gyro_x_phy * ctx.gyro_cfg.correction_coefficient;
    let gyro_x_phy = ctx.gyro_cfg.fir.filter(gyro_x_phy);

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

fn idle(
    ctx: &mut ControlContext,
    ls_enable: bool,
    lf_enable: bool,
    rf_enable: bool,
    rs_enable: bool,
) -> anyhow::Result<State> {
    loop {
        let cmd = ctx.command_rx.try_recv();
        if cmd.is_ok() {
            match cmd.unwrap() {
                Command::GyroCalibration => {
                    return Ok(State::GyroCalibration);
                }
                Command::SetActivateWallSensor(ls, lf, rf, rs) => {
                    return Ok(State::Idle(ls, lf, rf, rs));
                }
            }
        } else {
            measure(ctx, ls_enable, lf_enable, rf_enable, rs_enable)?;
            sync_ms();
        }
    }
}

fn gyro_calibration(ctx: &mut ControlContext) -> anyhow::Result<State> {
    // Measure gyro offset
    let mut gyro_offset = 0.0;
    led::on(Blue)?;
    {
        let mut imu = ctx.ods.imu.lock().unwrap();
        imu.gyro_x_offset = 0.0;
    }

    for _ in 0..1000 {
        measure(ctx, false, false, false, false)?;
        {
            let imu = ctx.ods.imu.lock().unwrap();
            gyro_offset += imu.gyro_x_phy as f32;
        }
        sync_ms();
    }

    {
        let mut imu = ctx.ods.imu.lock().unwrap();
        imu.gyro_x_offset = gyro_offset / 1000.0;
    }
    ctx.response_tx.send(Response::Done)?;
    led::off(Blue)?;
    Ok(State::Idle(false, false, false, false))
}

pub fn init(
    config: &config::YamlConfig,
    ods: &Arc<ods::Ods>,
    log_tx: Sender<log_thread::LogCommand>,
) -> anyhow::Result<(Sender<Command>, Receiver<Response>)> {
    // Message queues
    let (tx_for_ope, rx): (Sender<Command>, Receiver<Command>) = mpsc::channel();
    let (tx, rx_for_ope): (Sender<Response>, Receiver<Response>) = mpsc::channel();
    let mut state = State::Idle(false, false, false, false);

    let mut ctx = ControlContext {
        ods: ods.clone(),
        response_tx: tx,
        log_tx: log_tx,
        command_rx: rx,
        ws_cfg: WsConfig {
            led_rise_time: config.load_i64("led_rise_time", 30) as u32,
            rs_threshold: config.load_i64("rs_threshold", 200) as u16,
            rf_threshold: config.load_i64("rf_threshold", 50) as u16,
            lf_threshold: config.load_i64("lf_threshold", 400) as u16,
            ls_threshold: config.load_i64("ls_threshold", 130) as u16,
        },
        gyro_cfg: GyroConfig {
            fir: FIR::new(config.load_vec_f32("gyro_fir_coefficients", vec![1.0])),
            correction_table: config.load_vec_i16_f32(
                "gyro_correction_table",
                vec![(-32768, -2293.76), (32767, 2293.69)],
            ),
            correction_coefficient: config.load_f64("gyro_correction_coefficient", 1.0) as f32,
        },
        battery_cfg: BatteryConfig {
            correction_table: config
                .load_vec_i16_f32("battery_correction_table", vec![(355, 5.0), (688, 9.0)]),
        },
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
            match state {
                State::Idle(ls, lf, rf, rs) => {
                    state = idle(&mut ctx, ls, lf, rf, rs)?;
                }
                State::GyroCalibration => {
                    state = gyro_calibration(&mut ctx)?;
                }
            }
        }
    })?;

    Ok((tx_for_ope, rx_for_ope))
}
