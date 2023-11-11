use crate::config;
use crate::encoder;
use crate::imu;
use crate::led::{self, LedColor::*};
use crate::log_thread;
use crate::misc::{correct_value, FIR};
use crate::motor;
use crate::ods;
use crate::ods::MicromouseState;
use crate::pid;
use crate::timer_interrupt::{sync_ms, wait_us};
use crate::trajectory::{self, GenerateTrajectory};
use crate::wall_sensor;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;

struct WsConfig {
    led_rise_time: u32,
}

struct GyroConfig {
    fir: FIR<f32>,
    correction_table: Vec<(i16, f32)>,
}

struct EncoderConfig {
    wheel_diameter: f32,
    gear_ratio: f32,
}

struct BatteryConfig {
    correction_table: Vec<(i16, f32)>,
}

struct SpeedConfig {
    velocity: f32,
    acceleration: f32,
    deceleration: f32,
}

struct ControlContext {
    ods: Arc<ods::Ods>,
    log_tx: Sender<log_thread::LogCommand>,

    response_tx: Sender<Response>,
    command_rx: Receiver<Command>,
    ws_cfg: WsConfig,
    gyro_cfg: GyroConfig,
    enc_cfg: EncoderConfig,
    battery_cfg: BatteryConfig,

    sr_omega_pid: pid::Pid,
    sr_position_x_pid: pid::Pid,
    sr_position_y_pid: pid::Pid,
    sr_velocity_pid: pid::Pid,
    sr_ff_rate: f32,

    speed: SpeedConfig,
}

#[derive(Debug, PartialEq, Clone, Copy)]
enum State {
    Idle,
    GyroCalibration,
    WallSensorActive,
    Forward(f32),
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Command {
    GyroCalibration,
    ActivateWallSensor,
    InactivateWallSensor,
    Forward(f32),
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Response {
    Done,
}

fn measure(ctx: &mut ControlContext, wall_sensor_active: bool) -> anyhow::Result<()> {
    let batt = wall_sensor::read_batt()?;
    let batt_phy = correct_value(&ctx.battery_cfg.correction_table.as_slice(), batt as i16);

    if wall_sensor_active {
        wall_sensor::on_ls()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let ls = wall_sensor::read_ls()?;

        wall_sensor::on_lf()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let lf = wall_sensor::read_lf()?;

        wall_sensor::on_rf()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let rf = wall_sensor::read_rf()?;

        wall_sensor::on_rs()?;
        wait_us(ctx.ws_cfg.led_rise_time);
        let rs = wall_sensor::read_rs()?;

        let mut ws = ctx.ods.wall_sensor.lock().unwrap();
        (*ws).ls_raw = Some(ls);
        (*ws).lf_raw = Some(lf);
        (*ws).rf_raw = Some(rf);
        (*ws).rs_raw = Some(rs);
        (*ws).batt_raw = batt;
        (*ws).batt_phy = batt_phy;
    } else {
        let mut ws = ctx.ods.wall_sensor.lock().unwrap();
        (*ws).ls_raw = None;
        (*ws).lf_raw = None;
        (*ws).rf_raw = None;
        (*ws).rs_raw = None;
        (*ws).batt_raw = batt;
        (*ws).batt_phy = batt_phy;
    }

    wall_sensor::off()?;

    let gyro_x = imu::read()?;

    let encoder_l = encoder::read_l()?;
    let encoder_r = encoder::read_r()?;

    let gyro_x_phy = correct_value(&ctx.gyro_cfg.correction_table.as_slice(), gyro_x);
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

fn reset_micromouse_state(ctx: &mut ControlContext) {
    let mut micromouse = ctx.ods.micromouse.lock().unwrap();
    *micromouse = MicromouseState::default();
}

fn update(ctx: &mut ControlContext) {
    let enc_diff = {
        let enc = ctx.ods.encoder.lock().unwrap();
        (enc.l_diff as f32 + enc.r_diff as f32) / 2.0
    };
    let velocity =
        enc_diff / ctx.enc_cfg.gear_ratio * ctx.enc_cfg.wheel_diameter * std::f32::consts::PI
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

fn gyro_calibration(ctx: &mut ControlContext) -> anyhow::Result<State> {
    // Measure gyro offset
    let mut gyro_offset = 0.0;
    led::on(Blue)?;
    {
        let mut imu = ctx.ods.imu.lock().unwrap();
        imu.gyro_x_offset = 0.0;
    }

    for _ in 0..1000 {
        measure(ctx, false)?;
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
    Ok(State::Idle)
}

fn forward(ctx: &mut ControlContext, distance: f32) -> anyhow::Result<State> {
    let omega_target = 0.0;

    let x_t = distance;
    let v_i = 0.0;
    let v_l = ctx.speed.velocity;
    let v_f = 0.0;
    let a_a = ctx.speed.acceleration;
    let a_d = ctx.speed.deceleration;
    let mut trajectory = trajectory::ForwardTrajectory::new(x_t, v_i, v_l, v_f, a_a, a_d);

    ctx.sr_omega_pid.reset();
    ctx.sr_position_x_pid.reset();
    ctx.sr_position_y_pid.reset();
    ctx.sr_velocity_pid.reset();

    led::on(Blue)?;
    reset_micromouse_state(ctx);
    motor::set_l(0.0);
    motor::set_r(0.0);
    motor::enable(true);

    let mut ms_counter = crate::timer_interrupt::get_ms() - 1;

    ctx.log_tx.send(log_thread::LogCommand::Start)?;

    loop {
        measure(ctx, true)?;
        update(ctx);

        let is_end = trajectory.step();
        let target_x = trajectory.x_current;
        let target_v = trajectory.v_current;

        let micromouse = {
            let micromouse = ctx.ods.micromouse.lock().unwrap();
            micromouse.clone()
        };

        let omega_error = omega_target - micromouse.omega;
        let omega_ctrl = ctx.sr_omega_pid.update(omega_error);

        let position_x_error = target_x - micromouse.x;
        let position_x_ctrl = ctx.sr_position_x_pid.update(position_x_error);

        let position_y_error = 0.0 - micromouse.y;
        let position_y_ctrl = ctx.sr_position_y_pid.update(position_y_error);

        let velocity_error = target_v - micromouse.v;
        let velocity_ctrl = ctx.sr_velocity_pid.update(velocity_error);

        let ff_ctrl = ctx.sr_ff_rate * target_v;

        let motor_l = ff_ctrl + position_x_ctrl - position_y_ctrl + velocity_ctrl - omega_ctrl;
        let motor_r = ff_ctrl + position_x_ctrl + position_y_ctrl + velocity_ctrl + omega_ctrl;

        motor::set_l(motor_l * 1.3);
        motor::set_r(motor_r);

        let current_ms = crate::timer_interrupt::get_ms();
        if current_ms - ms_counter > 1 {
            let _ = crate::led::on(crate::led::LedColor::Red);
        } else {
            let _ = crate::led::off(crate::led::LedColor::Red);
        }

        ms_counter = current_ms;

        {
            let mut log = ctx.ods.log.lock().unwrap();
            if ms_counter % 5 == 0 && log.len() < log_thread::LOG_SIZE {
                log.push(log_thread::Log {
                    current_ms,
                    target_x,
                    current_x: micromouse.x,
                    target_v,
                    current_v: micromouse.v,
                    current_y: micromouse.y,
                    current_theta: micromouse.theta,
                    motor_l,
                    motor_r,
                    ff_ctrl,
                    position_x_ctrl,
                    position_y_ctrl,
                    omega_ctrl,
                });
            }
        }

        if is_end {
            motor::set_l(0.0);
            motor::set_r(0.0);
            motor::enable(false);
            break;
        }

        sync_ms();
    }
    ctx.log_tx.send(log_thread::LogCommand::Save)?;
    ctx.response_tx.send(Response::Done)?;
    led::off(Blue)?;
    Ok(State::Idle)
}

fn idle(ctx: &mut ControlContext, wall_sensor_active: bool) -> anyhow::Result<State> {
    loop {
        let cmd = ctx.command_rx.try_recv();
        if cmd.is_ok() {
            match cmd.unwrap() {
                Command::GyroCalibration => {
                    return Ok(State::GyroCalibration);
                }
                Command::ActivateWallSensor => {
                    return Ok(State::WallSensorActive);
                }
                Command::Forward(distance) => {
                    return Ok(State::Forward(distance));
                }
                Command::InactivateWallSensor => {
                    return Ok(State::Idle);
                }

                #[allow(unreachable_patterns)]
                _ => {
                    uprintln!("Invalid command {:?}", cmd.unwrap());
                    panic!("Invalid command {:?}", cmd.unwrap());
                }
            }
        }
        measure(ctx, wall_sensor_active)?;
        update(ctx);
        sync_ms();
    }
}

pub fn init(
    config: &config::YamlConfig,
    ods: &Arc<ods::Ods>,
    log_tx: Sender<log_thread::LogCommand>,
) -> anyhow::Result<(Sender<Command>, Receiver<Response>)> {
    // Message queues
    let (tx_for_ope, rx): (Sender<Command>, Receiver<Command>) = mpsc::channel();
    let (tx, rx_for_ope): (Sender<Response>, Receiver<Response>) = mpsc::channel();
    let mut state = State::Idle;

    // Load configurations
    // PID controller for search run
    let p = config.load_f64("sr_omega_p", 0.5) as f32;
    let i = config.load_f64("sr_omega_i", 0.0) as f32;
    let d = config.load_f64("sr_omega_d", 0.5) as f32;
    let i_limit = config.load_f64("sr_omega_i_limit", 0.5) as f32;
    let sr_omega_pid = pid::Pid::new(p, i, d, i_limit);

    let p = config.load_f64("sr_position_x_p", 0.5) as f32;
    let i = config.load_f64("sr_position_x_i", 0.0) as f32;
    let d = config.load_f64("sr_position_x_d", 0.5) as f32;
    let i_limit = config.load_f64("sr_position_x_i_limit", 0.5) as f32;
    let sr_position_x_pid = pid::Pid::new(p, i, d, i_limit);

    let p = config.load_f64("sr_position_y_p", 0.5) as f32;
    let i = config.load_f64("sr_position_y_i", 0.0) as f32;
    let d = config.load_f64("sr_position_y_d", 0.5) as f32;
    let i_limit = config.load_f64("sr_position_y_i_limit", 0.5) as f32;
    let sr_position_y_pid = pid::Pid::new(p, i, d, i_limit);

    // velocity PID
    let p = config.load_f64("sr_velocity_p", 0.5) as f32;
    let i = config.load_f64("sr_velocity_i", 0.0) as f32;
    let d = config.load_f64("sr_velocity_d", 0.5) as f32;
    let i_limit = config.load_f64("sr_velocity_i_limit", 0.5) as f32;
    let sr_velocity_pid = pid::Pid::new(p, i, d, i_limit);

    let spd_cfg: SpeedConfig = SpeedConfig {
        velocity: config.load_f64("sr_velocity", 0.1) as f32,
        acceleration: config.load_f64("sr_acceleration", 0.1) as f32,
        deceleration: config.load_f64("sr_deceleration", 0.1) as f32,
    };

    let sr_ff_rate = config.load_f64("sr_ff_rate", 0.1) as f32;

    let mut ctx = ControlContext {
        ods: ods.clone(),
        response_tx: tx,
        log_tx: log_tx,
        command_rx: rx,
        ws_cfg: WsConfig {
            led_rise_time: config.load_i64("led_rise_time", 30) as u32,
        },
        gyro_cfg: GyroConfig {
            fir: FIR::new(config.load_vec_f32(
                "gyro_fir_coefficients",
                vec![
                    -2.494829972812401e-18,
                    -7.851195903558143e-03,
                    4.014735544403485e-02,
                    -1.032535402203297e-01,
                    1.706609016841135e-01,
                    8.000000000000000e-01,
                    1.706609016841135e-01,
                    -1.032535402203297e-01,
                    4.014735544403485e-02,
                    -7.851195903558143e-03,
                    -2.494829972812401e-18,
                ],
            )),
            correction_table: config.load_vec_i16_f32(
                "gyro_correction_table",
                vec![(-32768, -2293.76), (32767, 2293.69)],
            ),
        },
        enc_cfg: EncoderConfig {
            wheel_diameter: config.load_f64("wheel_diameter", 13.0) as f32,
            gear_ratio: config.load_f64("gear_ratio", 1.5) as f32,
        },
        battery_cfg: BatteryConfig {
            correction_table: config
                .load_vec_i16_f32("battery_correction_table", vec![(355, 5.0), (688, 9.0)]),
        },
        sr_omega_pid,
        sr_position_x_pid: sr_position_x_pid,
        sr_position_y_pid: sr_position_y_pid,
        sr_velocity_pid: sr_velocity_pid,
        sr_ff_rate: sr_ff_rate,

        speed: spd_cfg,
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
                State::Idle => {
                    state = idle(&mut ctx, false)?;
                }
                State::GyroCalibration => {
                    state = gyro_calibration(&mut ctx)?;
                }
                State::WallSensorActive => {
                    state = idle(&mut ctx, true)?;
                }
                State::Forward(distance) => {
                    state = forward(&mut ctx, distance)?;
                }
            }
        }
    })?;

    Ok((tx_for_ope, rx_for_ope))
}
