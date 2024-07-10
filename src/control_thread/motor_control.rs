use crate::control_thread::{self, ControlContext};
use crate::led::{
    self,
    LedColor::{Green, Red},
};
use crate::mm_const;
use crate::motor;
use crate::ods::MicromouseState;
use crate::pid;
use crate::timer_interrupt;
use mm_maze::maze::Wall;

fn calc_duty(micromouse: &MicromouseState, voltage: f32) -> f32 {
    // battery voltage
    let vb = micromouse.v_batt;
    // voltage[V] -> duty[%]
    (voltage / vb) * 100.0
}

pub(super) enum SequenceResult {
    Continue(f32),
    End(f32),
}

pub(super) trait VoltageSequence {
    fn next(&mut self, current_position: f32) -> SequenceResult;
}

// +++++++ StartSequence +++++++
pub(super) struct StartSequence {
    i: u32,
    distance: f32,
    base_voltage: f32,
    slope: f32,
    brake_count: u32,
}

impl StartSequence {
    const STARTUP_TIME: u32 = 100;
    pub fn new(distance: f32, base_voltage: f32) -> Self {
        Self {
            i: 0,
            distance,
            base_voltage,
            slope: base_voltage / (Self::STARTUP_TIME as f32),
            brake_count: 0,
        }
    }

    #[allow(dead_code)]
    fn reset(&mut self) {
        self.i = 0;
        self.brake_count = 0;
    }
}

impl VoltageSequence for StartSequence {
    fn next(&mut self, current_position: f32) -> SequenceResult {
        if self.i < 100 {
            let v = self.i as f32 * self.slope;
            self.i += 1;
            SequenceResult::Continue(v)
        } else {
            if current_position > self.distance {
                SequenceResult::End(self.base_voltage)
            } else {
                SequenceResult::Continue(self.base_voltage)
            }
        }
    }
}

// +++++++ ConstSequence +++++++
struct ConstSequence {
    distance: f32,
    base_volt: f32,
}

impl ConstSequence {
    pub fn new(distance: f32, base_volt: f32) -> Self {
        Self {
            distance,
            base_volt,
        }
    }

    #[allow(dead_code)]
    pub fn set_distance(&mut self, distance: f32) {
        self.distance = distance;
    }

    #[allow(dead_code)]
    pub fn set_base_volt(&mut self, base_volt: f32) {
        self.base_volt = base_volt;
    }
}

impl VoltageSequence for ConstSequence {
    fn next(&mut self, current_position: f32) -> SequenceResult {
        if current_position > self.distance {
            SequenceResult::End(self.base_volt)
        } else {
            SequenceResult::Continue(self.base_volt)
        }
    }
}

// +++++++ StopSequence +++++++
struct StopSequence {
    distance: f32,
    base_volt: f32,
    brake_count: u16,
}

impl StopSequence {
    pub fn new(distance: f32, base_volt: f32) -> Self {
        Self {
            distance,
            base_volt,
            brake_count: 0,
        }
    }

    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.brake_count = 0;
    }

    #[allow(dead_code)]
    pub fn set_distance(&mut self, distance: f32) {
        self.distance = distance;
    }
}

impl VoltageSequence for StopSequence {
    fn next(&mut self, current_position: f32) -> SequenceResult {
        if self.brake_count > 0 {
            self.brake_count += 1;
            if self.brake_count > 100 {
                SequenceResult::End(0.0)
            } else {
                SequenceResult::Continue(0.0)
            }
        } else {
            let v = (self.distance - current_position) * 10.0;
            if current_position > self.distance {
                self.brake_count = 1;
            }
            let v = v.min(self.base_volt).max(0.15);
            SequenceResult::Continue(v)
        }
    }
}

fn go(
    ctx: &mut ControlContext,
    sequence: &mut dyn VoltageSequence,
    notify_distance: Option<f32>,
    enable_wall_pid: bool,
) -> anyhow::Result<()> {
    let mut need_request = notify_distance.is_some();

    let mut current_position;
    let mut enable_wall_edge_r;
    let mut enable_wall_edge_l;

    {
        let ods = ctx.ods.lock().unwrap();
        current_position = ods.micromouse.y;
        enable_wall_edge_l = ods.micromouse.ls_wall.to_bool() && ctx.config.ws_cfg.wall_edge_enable;
        enable_wall_edge_r = ods.micromouse.rs_wall.to_bool() && ctx.config.ws_cfg.wall_edge_enable;
    };
    let mut flag = true;

    while flag {
        let target_v = match sequence.next(current_position) {
            SequenceResult::Continue(v) => v,
            SequenceResult::End(v) => {
                flag = false;
                v
            }
        };
        {
            let mut ods = ctx.ods.lock().unwrap();
            ods.micromouse.target_v = target_v;
        }
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);

        current_position = micromouse.y;

        // Wall edge correction
        if (enable_wall_edge_r) && (micromouse.y > 0.035) && (micromouse.rs < ctx.rs_ref / 3) {
            enable_wall_edge_r = false;
            enable_wall_edge_l = false;
            current_position = ctx.config.ws_cfg.wall_edge_position;
        }
        if (enable_wall_edge_l) && (micromouse.y > 0.035) && (micromouse.ls < ctx.ls_ref / 3) {
            enable_wall_edge_r = false;
            enable_wall_edge_l = false;
            current_position = ctx.config.ws_cfg.wall_edge_position;
        }

        let fb_v = ctx.v_pid.update(target_v - micromouse.v);

        // Set target theta by wall sensor
        let ws_error = if enable_wall_pid {
            match (micromouse.ls_wall, micromouse.rs_wall) {
                (Wall::Present, Wall::Present) => {
                    let l_error = ctx.ls_ref as i16 - micromouse.ls as i16;
                    let r_error = micromouse.rs as i16 - ctx.rs_ref as i16;
                    Some(r_error + l_error)
                }
                (Wall::Present, Wall::Absent) => Some(ctx.ls_ref as i16 - micromouse.ls as i16),
                (Wall::Absent, Wall::Present) => Some(micromouse.rs as i16 - ctx.rs_ref as i16),
                (_, _) => None,
            }
        } else {
            None
        };

        let fb_theta = if let Some(ws_error) = ws_error {
            let ws_error = ws_error.max(-40).min(40);
            ctx.position_reset_count += 1;
            ctx.wall_pid.update(ws_error as f32 / 1000.0)
        } else {
            ctx.position_reset_count = 0;
            ctx.theta_pid
                .update(std::f32::consts::PI / 2.0 - micromouse.theta)
        };

        // Update MicromouseState
        {
            let mut ods = ctx.ods.lock().unwrap();
            ods.micromouse.y = current_position;

            if ctx.position_reset_count > 500 {
                ods.micromouse.x = mm_const::BLOCK_LENGTH / 2.0;
                ods.micromouse.theta = std::f32::consts::PI / 2.0;
                ctx.position_reset_count = 0;
            }
        }

        // Angle feedback
        let fb_omega = ctx.omega_pid.update(0.0 - micromouse.omega);

        // Set duty
        let duty_r = calc_duty(&micromouse, fb_v + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, fb_v - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);

        ctx.log();

        if need_request {
            let nd = notify_distance.unwrap();
            if current_position > nd {
                ctx.request_command();
                need_request = false;
            }
        }

        timer_interrupt::sync_ms();
    }
    control_thread::set_motor_duty(ctx, 0.0, 0.0);

    Ok(())
}

pub(super) fn stop(
    ctx: &mut ControlContext,
    distance: f32,
    command_request: bool,
) -> anyhow::Result<()> {
    led::on(Red)?;
    let mut seq = StopSequence::new(distance, ctx.config.search_ctrl_cfg.vel_fwd);
    go(ctx, &mut seq, None, false)?;

    control_thread::measure(ctx)?;
    control_thread::update(ctx);
    if command_request {
        ctx.request_command();
    }
    control_thread::set_motor_duty(ctx, 0.0, 0.0);

    ctx.log();

    ctx.reset_controllers();

    timer_interrupt::sync_ms();

    led::off(Red)?;
    Ok(())
}

pub(super) fn start(ctx: &mut ControlContext, distance: f32) -> anyhow::Result<()> {
    led::on(Red)?;
    let mut seq = StartSequence::new(distance, ctx.config.search_ctrl_cfg.vel_fwd);
    go(
        ctx,
        &mut seq,
        Some(distance - (mm_const::BLOCK_LENGTH - ctx.config.judge_position)),
        false,
    )?;

    {
        let mut ods = ctx.ods.lock().unwrap();
        ods.micromouse.y -= mm_const::BLOCK_LENGTH;
    }
    led::off(Red)?;
    Ok(())
}

pub(super) fn reset_controller(ctx: &mut ControlContext) {
    ctx.theta_pid = pid::Pid::new(&ctx.config.search_ctrl_cfg.theta_pid);
    ctx.omega_pid = pid::Pid::new(&ctx.config.search_ctrl_cfg.omega_pid);
    ctx.v_pid = pid::Pid::new(&ctx.config.search_ctrl_cfg.v_pid);
    ctx.pos_pid = pid::Pid::new(&ctx.config.search_ctrl_cfg.pos_pid);
    ctx.wall_pid = pid::Pid::new(&ctx.config.search_ctrl_cfg.wall_pid);

    ctx.ws_ena = true;
    let mut ls: i32 = 0;
    let mut rs: i32 = 0;
    control_thread::measure(ctx).unwrap();
    for _ in 0..100 {
        control_thread::measure(ctx).unwrap();
        {
            let ods = ctx.ods.lock().unwrap();

            ls += ods.wall_sensor.ls_raw.unwrap() as i32;
            rs += ods.wall_sensor.rs_raw.unwrap() as i32;
        }
    }
    ctx.ls_ref = (ls / 100) as u16;
    ctx.rs_ref = (rs / 100) as u16;

    ctx.log_msg(format!("ls_ref: {}, rs_ref: {}", ctx.ls_ref, ctx.rs_ref));

    control_thread::reset_micromouse_state(ctx);
    motor::set_l(0.0);
    motor::set_r(0.0);
    motor::enable(true);
    timer_interrupt::sync_ms();
}

pub(super) fn forward(ctx: &mut ControlContext, distance: f32) -> anyhow::Result<()> {
    // constant speed
    led::on(Green)?;
    let mut seq = ConstSequence::new(distance, ctx.config.search_ctrl_cfg.vel_fwd);
    let nb = Some(distance - (mm_const::BLOCK_LENGTH - ctx.config.judge_position));
    go(ctx, &mut seq, nb, true)?;
    {
        let mut ods = ctx.ods.lock().unwrap();
        ods.micromouse.y -= mm_const::BLOCK_LENGTH;
    }
    led::off(Green)?;
    Ok(())
}

pub(super) fn test(ctx: &mut ControlContext) -> anyhow::Result<()> {
    pivot(ctx, std::f32::consts::PI / 2.0, 0.4)?;
    pivot(ctx, std::f32::consts::PI / 2.0, 0.4)?;
    pivot(ctx, std::f32::consts::PI / 2.0, 0.4)?;
    pivot(ctx, std::f32::consts::PI / 2.0, 0.4)?;
    pivot(ctx, std::f32::consts::PI / 2.0, 0.4)?;
    pivot(ctx, std::f32::consts::PI / 2.0, 0.4)?;
    pivot(ctx, std::f32::consts::PI / 2.0, 0.4)?;
    pivot(ctx, std::f32::consts::PI / 2.0, 0.4)?;
    pivot(ctx, std::f32::consts::PI / 2.0, 0.4)?;
    pivot(ctx, std::f32::consts::PI / 2.0, 0.4)
}

pub(super) fn turn_left(ctx: &mut ControlContext) -> anyhow::Result<()> {
    stop(ctx, mm_const::BLOCK_LENGTH / 2.0, false)?;
    nop(ctx, 0.1)?;
    pivot(ctx, std::f32::consts::PI / 2.0, 0.2)?;
    nop(ctx, 0.1)?;
    forward(ctx, mm_const::BLOCK_LENGTH)
}

pub(super) fn turn_right(ctx: &mut ControlContext) -> anyhow::Result<()> {
    stop(ctx, mm_const::BLOCK_LENGTH / 2.0, false)?;
    nop(ctx, 0.1)?;
    pivot(ctx, -std::f32::consts::PI / 2.0, 0.2)?;
    nop(ctx, 0.1)?;
    forward(ctx, mm_const::BLOCK_LENGTH)
}

pub(super) fn turn_back(ctx: &mut ControlContext) -> anyhow::Result<()> {
    stop(ctx, mm_const::BLOCK_LENGTH / 2.0, false)?;
    nop(ctx, 0.1)?;
    pivot(ctx, -std::f32::consts::PI / 2.0, 0.2)?;
    pivot(ctx, -std::f32::consts::PI / 2.0, 0.2)?;
    nop(ctx, 0.1)?;
    forward(ctx, mm_const::BLOCK_LENGTH)
}

pub(super) fn pivot(ctx: &mut ControlContext, angle: f32, duration: f32) -> anyhow::Result<()> {
    let original_angle = {
        let ods = ctx.ods.lock().unwrap();
        ods.micromouse.theta
    };
    let mut target_omega = angle / duration;
    let mut target_theta;

    let mut time = 0.0;
    let total_duration = duration * 2.0;

    let mut stop = false;

    while time < total_duration {
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);

        if stop {
            target_omega = 0.0;
            target_theta = original_angle + angle;
        } else {
            if time >= duration {
                stop = true;
            }
            target_theta = original_angle + target_omega * time;
        };

        let fb_theta = ctx.theta_pid.update(target_theta - micromouse.theta);
        let fb_omega = ctx.omega_pid.update(target_omega - micromouse.omega);

        let fb_v = ctx.v_pid.update(0.0 - micromouse.v);

        let duty_r = calc_duty(&micromouse, fb_v + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, fb_v - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);
        ctx.log();
        time += mm_const::DT;
        timer_interrupt::sync_ms();
    }
    control_thread::set_motor_duty(ctx, 0.0, 0.0);

    {
        // Reset position
        let mut ods = ctx.ods.lock().unwrap();
        ods.micromouse.theta = std::f32::consts::PI / 2.0;
        ods.micromouse.x = mm_const::BLOCK_LENGTH / 2.0;
        ods.micromouse.y = mm_const::BLOCK_LENGTH / 2.0;
    }

    ctx.reset_controllers();

    Ok(())
}

pub(super) fn nop(ctx: &mut ControlContext, duration: f32) -> anyhow::Result<()> {
    let mut time = 0.0;
    let total_duration = duration * 1.5;

    let target_theta = std::f32::consts::PI / 2.0;
    let target_omega = 0.0;

    while time < total_duration {
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);

        let fb_theta = ctx.theta_pid.update(target_theta - micromouse.theta);
        let fb_omega = ctx.omega_pid.update(target_omega - micromouse.omega);

        let fb_v = ctx.v_pid.update(0.0 - micromouse.v);

        let duty_r = calc_duty(&micromouse, fb_v + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, fb_v - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);
        ctx.log();
        time += mm_const::DT;
        timer_interrupt::sync_ms();
    }

    {
        // Reset position
        let mut ods = ctx.ods.lock().unwrap();
        ods.micromouse.theta = std::f32::consts::PI / 2.0;
        ods.micromouse.x = mm_const::BLOCK_LENGTH / 2.0;
        ods.micromouse.y = mm_const::BLOCK_LENGTH / 2.0;
    }

    ctx.reset_controllers();

    Ok(())
}
