use crate::control_thread::{self, ControlContext, Response};
use crate::led::{
    self,
    LedColor::{Green, Red},
};
use crate::mm_const;
use crate::motor;
use crate::ods::{self, MicromouseState};
use crate::pid;
use crate::timer_interrupt;
use mm_maze::maze::Wall;
use mm_traj;

fn calc_duty(_micromouse: &MicromouseState, voltage: f32) -> f32 {
    // battery voltage
    let vb = 7.0; //micromouse.v_batt;
                  // voltage[V] -> duty[%]
    (voltage / vb) * 100.0
}

fn calc_diff(xa: f32, ya: f32, xb: f32, yb: f32, theta: f32) -> f32 {
    let delta_x = xb - xa;
    let delta_y = yb - ya;
    let diff = delta_x * theta.cos() + delta_y * theta.sin();
    diff
}

fn make_target_from_ods(ctx: &ControlContext) -> mm_traj::State {
    let ods = ctx.ods.lock().unwrap();
    mm_traj::State {
        x: ods.micromouse.target_x,
        y: ods.micromouse.target_y,
        theta: ods.micromouse.target_theta,
        omega: ods.micromouse.target_omega,
        v: ods.micromouse.target_v,
        a: ods.micromouse.target_a,
    }
}

fn go(
    ctx: &mut ControlContext,
    distance: f32,
    final_velocity: f32,
    notify_distance: Option<f32>,
    enable_wall_pid: bool,
) -> anyhow::Result<()> {
    let mut need_request = notify_distance.is_some(); // if notify_distance is Some, then need_request is true
    let target = make_target_from_ods(ctx);
    let mut use_wall_pid = enable_wall_pid;
    let start_position = mm_const::BLOCK_LENGTH - distance;

    let mut straight = mm_traj::Straight::new(target, distance, final_velocity);
    let mut event = ods::Event::Go;
    for _ in 0..450 {
        led::on(Red)?;
        control_thread::measure(ctx)?;
        let mut micromouse = control_thread::update(ctx);
        control_thread::update_target(ctx, &target);

        let (oy, ox) = straight.origin();
        let current_position =
            ((micromouse.x - ox).powi(2) + (micromouse.y - oy).powi(2)).sqrt() + start_position;

        // Set target theta by wall sensor
        let ws_error = if use_wall_pid {
            match (micromouse.ls_wall, micromouse.rs_wall) {
                (Wall::Present, Wall::Present) => {
                    let l_error = ctx.ls_ref - micromouse.ls as i16;
                    let r_error = micromouse.rs as i16 - ctx.rs_ref;
                    Some(r_error + l_error)
                }
                // (Wall::Present, Wall::Absent) => Some((ctx.ls_ref - micromouse.ls as i16) * 2),
                // (Wall::Absent, Wall::Present) => Some((micromouse.rs as i16 - ctx.rs_ref) * 2),
                (_, _) => None,
            }
        } else {
            None
        };

        let mut fb_ws = 0;
        let fb_theta = if let Some(ws_error) = ws_error {
            if (ws_error.abs() as f32) < 3.0 {
                ctx.position_reset_count += 1;

                let mut ods = ctx.ods.lock().unwrap();
                if ctx.position_reset_count >= 100 {
                    led::on(Green)?;
                    event = ods::Event::PositionReset;
                    if target.theta.sin().abs() > 0.5 {
                        // +-Y direction
                        ods.micromouse.x = target.x;
                        micromouse.x = target.x;
                        ods.micromouse.theta = target.theta;
                        micromouse.theta = target.theta;
                    } else {
                        // +-X direction
                        ods.micromouse.y = target.y;
                        micromouse.y = target.y;
                        ods.micromouse.theta = target.theta;
                        micromouse.theta = target.theta;
                    }
                    ctx.position_reset_count = 0;
                }
            } else {
                ctx.position_reset_count = 0;
            }
            fb_ws = 1;
            let ws_error = ws_error.max(-40).min(40);
            ctx.wall_pid.update(ws_error as f32 / 1000.0)
        } else {
            ctx.position_reset_count = 0;
            ctx.theta_pid.update(target.theta - micromouse.theta)
        };

        // Angle feedback
        let fb_omega = ctx.omega_pid.update(target.omega - micromouse.omega);

        // Position feedback
        let diff_pos = calc_diff(micromouse.x, micromouse.y, target.x, target.y, target.theta);
        let target_v = ctx.pos_pid.update(diff_pos);
        // Velocity feedback
        let fb_v = 0.5; //ctx.v_pid.update(target_v - micromouse.v);

        // Set duty
        let duty_r = calc_duty(&micromouse, fb_v + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, fb_v - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);

        if need_request == true {
            if current_position > notify_distance.unwrap() + start_position {
                ctx.response_tx.send(Response::CommandRequest).unwrap();
                event = ods::Event::CommandRequest;
                need_request = false;
            }
        }

        {
            let mut ods = ctx.ods.lock().unwrap();
            ods.micromouse.event = event;
            ods.micromouse.ws_error = ws_error.unwrap_or(0);
            ods.micromouse.fb_ws = fb_ws;
            ods.micromouse.fb_theta = fb_theta;
        }

        ctx.log();
        timer_interrupt::sync_ms();
        event = ods::Event::None;
        led::off(Red)?;
        led::off(Green)?;
    }
    control_thread::set_motor_duty(ctx, 0.0, 0.0);
    Ok(())
}

pub(super) fn stop(
    ctx: &mut ControlContext,
    distance: f32,
    command_request: bool,
) -> anyhow::Result<()> {
    // decelerate
    go(ctx, distance, 0.0, None, false)?;
    let target = make_target_from_ods(ctx);

    for _ in 0..100 {
        led::on(Red)?;
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);
        control_thread::update_target(ctx, &target);
        let fb_theta = ctx.theta_pid.update(target.theta - micromouse.theta);
        let fb_omega = ctx.omega_pid.update(target.omega - micromouse.omega);

        let diff_pos = calc_diff(micromouse.x, micromouse.y, target.x, target.y, target.theta);
        let target_v = ctx.pos_pid.update(diff_pos);
        let fb_v = ctx.v_pid.update(target_v - micromouse.v);
        let duty_r = calc_duty(&micromouse, fb_v + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, fb_v - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);
        ctx.log();
        led::off(Red)?;
        timer_interrupt::sync_ms();
    }
    control_thread::set_motor_duty(ctx, 0.0, 0.0);

    if command_request {
        ctx.response_tx.send(Response::CommandRequest).unwrap();
    }

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
    ctx.ls_ref = (ls / 100) as i16;
    ctx.rs_ref = (rs / 100) as i16;

    ctx.log_msg(format!("ls_ref: {}, rs_ref: {}", ctx.ls_ref, ctx.rs_ref));

    control_thread::reset_micromouse_state(ctx);
    motor::set_l(0.0);
    motor::set_r(0.0);
    motor::enable(true);
    timer_interrupt::sync_ms();
}

pub(super) fn forward(ctx: &mut ControlContext, distance: f32) -> anyhow::Result<()> {
    // constant speed
    go(
        ctx,
        distance,
        ctx.config.search_ctrl_cfg.vel_fwd,
        Some(distance - (mm_const::BLOCK_LENGTH - mm_const::JUDGE_POSITION)),
        true,
    )
}

pub(super) fn test(ctx: &mut ControlContext) -> anyhow::Result<()> {
    let target = mm_traj::State::default();
    motor::enable(true);
    for i in 0..1500 {
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);
        control_thread::update_target(ctx, &target);

        let base_duty = if i < 1000 { 1.0 } else { 0.0 };

        let fb_theta = ctx
            .theta_pid
            .update(std::f32::consts::PI - micromouse.theta);
        let fb_omega = ctx.omega_pid.update(0.0 - micromouse.omega);
        let duty_r = calc_duty(&micromouse, base_duty + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, base_duty - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);
        ctx.log();
        timer_interrupt::sync_ms();
    }
    motor::set_l(0.0);
    motor::set_r(0.0);
    motor::enable(false);
    ctx.response_tx.send(Response::CommandRequest).unwrap();
    Ok(())
}

pub(super) fn turn_left(ctx: &mut ControlContext) -> anyhow::Result<()> {
    stop(ctx, mm_const::BLOCK_LENGTH / 2.0, false)?;
    pivot(ctx, std::f32::consts::PI / 2.0, 0.35)?;
    forward(ctx, mm_const::BLOCK_LENGTH / 2.0)
}

pub(super) fn turn_right(ctx: &mut ControlContext) -> anyhow::Result<()> {
    stop(ctx, mm_const::BLOCK_LENGTH / 2.0, false)?;
    pivot(ctx, -std::f32::consts::PI / 2.0, 0.35)?;
    forward(ctx, mm_const::BLOCK_LENGTH / 2.0)
}

pub(super) fn turn_back(ctx: &mut ControlContext) -> anyhow::Result<()> {
    stop(ctx, mm_const::BLOCK_LENGTH / 2.0, false)?;
    pivot(ctx, std::f32::consts::PI, 0.7)?;
    forward(ctx, mm_const::BLOCK_LENGTH / 2.0)
}

pub(super) fn pivot(ctx: &mut ControlContext, angle: f32, duration: f32) -> anyhow::Result<()> {
    let target = make_target_from_ods(ctx);

    let mut pivot = mm_traj::Pivot::new(target, angle, duration);
    let mut event = ods::Event::Pivot;

    while let Some(target) = pivot.next() {
        led::on(Red)?;
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);
        control_thread::update_target(ctx, &target);

        let fb_theta = ctx.theta_pid.update(target.theta - micromouse.theta);
        let fb_omega = ctx.omega_pid.update(target.omega - micromouse.omega);

        let fb_v = ctx.v_pid.update(0.0 - micromouse.v);

        let duty_r = calc_duty(&micromouse, fb_v + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, fb_v - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);

        {
            let mut ods = ctx.ods.lock().unwrap();
            ods.micromouse.event = event;
        }
        event = ods::Event::None;

        ctx.log();
        led::off(Red)?;
        timer_interrupt::sync_ms();
    }

    let target = make_target_from_ods(ctx);

    for _ in 0..250 {
        led::on(Red)?;
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);
        control_thread::update_target(ctx, &target);
        let fb_theta = ctx.theta_pid.update(target.theta - micromouse.theta);
        let fb_omega = ctx.omega_pid.update(target.omega - micromouse.omega);

        let diff_pos = calc_diff(micromouse.x, micromouse.y, target.x, target.y, target.theta);
        let target_v = ctx.pos_pid.update(diff_pos);
        let fb_v = ctx.v_pid.update(target_v - micromouse.v);
        let duty_r = calc_duty(&micromouse, fb_v + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, fb_v - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);
        ctx.log();
        led::off(Red)?;
        timer_interrupt::sync_ms();
    }
    control_thread::set_motor_duty(ctx, 0.0, 0.0);

    Ok(())
}
