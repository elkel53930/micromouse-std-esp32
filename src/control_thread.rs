use esp_idf_hal::delay::FreeRtos;

use crate::config;
use crate::encoder;
use crate::imu;
use crate::led;
use crate::misc::{correct_value, FIR};
use crate::motor;
use crate::ods;
use crate::timer_interrupt::{sync_ms, wait_us};
use crate::wall_sensor;
use std::sync::mpsc::{self, Receiver, Sender};

use std::sync::Arc;

struct WsContext {
    led_rise_time: u32,
}

struct GyroContext {
    fir: FIR<f32>,
    correction_table: Vec<(i16, f32)>,
}

struct ControlContext {
    ods: Arc<ods::Ods>,
    response_tx: Sender<Response>,
    command_rx: Receiver<Command>,
    ws_ctx: WsContext,
    gyro_ctx: GyroContext,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum State {
    Idle,
    GyroCalibration,
    WallSensorActive,
    Forward,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Command {
    GyroCalibration,
    ActivateWallSensor,
    InactivateWallSensor,
    Forward,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Response {
    Done,
}

fn measure(ctx: &mut ControlContext, wall_sensor_active: bool) -> anyhow::Result<()> {
    let batt = wall_sensor::read_batt()?;

    if wall_sensor_active {
        wall_sensor::on_ls()?;
        wait_us(ctx.ws_ctx.led_rise_time);
        let ls = wall_sensor::read_ls()?;

        wall_sensor::on_lf()?;
        wait_us(ctx.ws_ctx.led_rise_time);
        let lf = wall_sensor::read_lf()?;

        wall_sensor::on_rf()?;
        wait_us(ctx.ws_ctx.led_rise_time);
        let rf = wall_sensor::read_rf()?;

        wall_sensor::on_rs()?;
        wait_us(ctx.ws_ctx.led_rise_time);
        let rs = wall_sensor::read_rs()?;

        let mut ws = ctx.ods.wall_sensor.lock().unwrap();
        (*ws).ls_raw = Some(ls);
        (*ws).lf_raw = Some(lf);
        (*ws).rf_raw = Some(rf);
        (*ws).rs_raw = Some(rs);
        (*ws).batt_raw = batt;
    } else {
        let mut ws = ctx.ods.wall_sensor.lock().unwrap();
        (*ws).ls_raw = None;
        (*ws).lf_raw = None;
        (*ws).rf_raw = None;
        (*ws).rs_raw = None;
        (*ws).batt_raw = batt;
    }

    wall_sensor::off()?;

    let gyro_x = imu::read()?;

    let encoder_l = encoder::read_l()?;
    let encoder_r = encoder::read_r()?;

    let gyro_x_phy = correct_value(&ctx.gyro_ctx.correction_table.as_slice(), gyro_x);
    let gyro_x_phy = ctx.gyro_ctx.fir.filter(gyro_x_phy);

    {
        let mut imu = ctx.ods.imu.lock().unwrap();
        imu.gyro_x_raw = gyro_x;
        imu.gyro_x_phy = gyro_x_phy - imu.gyro_x_offset;
    }
    {
        let mut enc = ctx.ods.encoder.lock().unwrap();
        enc.l_raw = encoder_l;
        enc.r_raw = encoder_r;
    }

    Ok(())
}

fn gyro_calibration(ctx: &mut ControlContext) -> anyhow::Result<State> {
    // Measure gyro offset
    let mut gyro_offset = 0.0;
    led::on(led::LedColor::Blue)?;
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
    led::off(led::LedColor::Blue)?;
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
                Command::Forward => {
                    return Ok(State::Forward);
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
        sync_ms();
    }
}

pub fn init(
    config: &config::YamlConfig,
    ods: &Arc<ods::Ods>,
) -> anyhow::Result<(Sender<Command>, Receiver<Response>)> {
    // Todo: Load configurations

    let (tx_for_ope, rx): (Sender<Command>, Receiver<Command>) = mpsc::channel();
    let (tx, rx_for_ope): (Sender<Response>, Receiver<Response>) = mpsc::channel();
    let mut state = State::Idle;

    let mut ctx = ControlContext {
        ods: ods.clone(),
        response_tx: tx,
        command_rx: rx,
        ws_ctx: WsContext {
            led_rise_time: config.load_i64("led_rise_time", 30) as u32,
        },
        gyro_ctx: GyroContext {
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
                State::Forward => {
                    state = State::Idle;
                }
            }
        }
    })?;

    Ok((tx_for_ope, rx_for_ope))
}
