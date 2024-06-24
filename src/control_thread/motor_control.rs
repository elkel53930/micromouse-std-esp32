use crate::control_thread::{self, ControlContext, Response};
use crate::motor;
use crate::ods::{self, MicromouseState};
use crate::pid;
use crate::timer_interrupt;
use mm_maze::maze::Wall;
use mm_traj::{self, TrajResult};

// velocity[m/s] -> voltage[V]
fn calc_ff_l(ctx: &ControlContext, vel: f32) -> f32 {
    let coeff = ctx.config.search_ctrl_cfg.ff_coeff_l;
    let offset = ctx.config.search_ctrl_cfg.ff_offset_l;
    let volt = vel * coeff + offset;
    volt.max(0.0).min(7.4)
}

// velocity[m/s] -> voltage[V]
fn calc_ff_r(ctx: &ControlContext, v: f32) -> f32 {
    let coeff = ctx.config.search_ctrl_cfg.ff_coeff_r;
    let offset = ctx.config.search_ctrl_cfg.ff_offset_r;
    let volt = v * coeff + offset;
    volt.max(0.0).min(7.4)
}

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

fn go(
    ctx: &mut ControlContext,
    distance: f32,
    final_velocity: f32,
    notify_distance: Option<f32>,
) -> anyhow::Result<()> {
    let mut need_request = notify_distance.is_some(); // if notify_distance is Some, then need_request is true
    let target = {
        let ods = ctx.ods.lock().unwrap();
        mm_traj::State {
            x: ods.micromouse.target_x,
            y: ods.micromouse.target_y,
            theta: ods.micromouse.target_theta,
            omega: 0.0,
            v: ods.micromouse.target_v,
            a: 0.0,
        }
    };

    let mut straight = mm_traj::Straight::new(target, distance, final_velocity);
    let mut flag = true;
    let mut event;
    while flag {
        event = ods::Event::None;
        let target = straight.proceed();
        // uprintln!("target={:?}", target);
        let target = match target {
            TrajResult::Done(target) => {
                flag = false;
                event = ods::Event::GoDone;
                target
            }
            TrajResult::Continue(target) => target,
        };

        crate::led::on(crate::led::LedColor::Red)?;
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);
        control_thread::update_target(ctx, &target);

        // Set target theta by wall sensor
        let ws_error = match (micromouse.ls_wall, micromouse.rs_wall) {
            (Wall::Present, Wall::Present) => micromouse.rs as i16 - micromouse.ls as i16,
            (Wall::Present, Wall::Absent) => (50 - micromouse.ls as i16) * 2,
            (Wall::Absent, Wall::Present) => (micromouse.rs as i16 - 50) * 2,
            (_, _) => 0,
        };
        let target_theta = target.theta + (ws_error as f32) * ctx.config.search_ctrl_cfg.ws_gain;

        // Angle feedback
        let fb_theta = ctx.theta_pid.update(target_theta - micromouse.theta);
        let fb_omega = ctx.omega_pid.update(target.omega - micromouse.omega);

        // Feedforward
        let ff_l = calc_ff_l(ctx, target.v);
        let ff_r = calc_ff_r(ctx, target.v);

        // Position feedback
        let diff_pos = calc_diff(micromouse.x, micromouse.y, target.x, target.y, target.theta);
        let diff_pos = ctx.pos_ave.update(diff_pos);
        let target_v = ctx.pos_pid.update(diff_pos);
        // Velocity feedback
        let fb_v = ctx.v_pid.update(target_v - micromouse.v);

        // Set duty
        let duty_r = calc_duty(&micromouse, ff_r + fb_v + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, ff_l + fb_v - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);

        if need_request == true {
            if let Some(notify_distance) = notify_distance {
                let (oy, ox) = straight.origin();
                let diff_from_origin =
                    ((micromouse.x - ox).powi(2) + (micromouse.y - oy).powi(2)).sqrt();
                if diff_from_origin > notify_distance {
                    ctx.response_tx.send(Response::CommandRequest).unwrap();
                    event = ods::Event::CommandRequest;
                    need_request = false;
                }
            }
        }

        {
            let mut ods = ctx.ods.lock().unwrap();
            ods.micromouse.event = event;
            ods.micromouse.ws_error = ws_error;
        }

        ctx.log();
        crate::led::off(crate::led::LedColor::Red)?;
        timer_interrupt::sync_ms();
    }
    Ok(())
}

pub(super) fn stop(ctx: &mut ControlContext, distance: f32) -> anyhow::Result<()> {
    // decelerate
    go(ctx, distance, 0.0, None)?;
    let target = {
        let ods = ctx.ods.lock().unwrap();
        mm_traj::State {
            x: ods.micromouse.target_x,
            y: ods.micromouse.target_y,
            theta: ods.micromouse.target_theta,
            omega: 0.0,
            v: ods.micromouse.target_v,
            a: 0.0,
        }
    };

    for _ in 0..200 {
        crate::led::on(crate::led::LedColor::Red)?;
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);
        control_thread::update_target(ctx, &target);
        let fb_theta = ctx.theta_pid.update(target.theta - micromouse.theta);
        let fb_omega = ctx.omega_pid.update(target.omega - micromouse.omega);

        let diff_pos = calc_diff(micromouse.x, micromouse.y, target.x, target.y, target.theta);
        let diff_pos = ctx.pos_ave.update(diff_pos);
        let target_v = ctx.pos_pid.update(diff_pos);
        let fb_v = ctx.v_pid.update(target_v - micromouse.v);
        let duty_r = calc_duty(&micromouse, fb_v + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, fb_v - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);
        ctx.log();
        crate::led::off(crate::led::LedColor::Red)?;
        timer_interrupt::sync_ms();
    }
    control_thread::set_motor_duty(ctx, 0.0, 0.0);
    ctx.stop_log();

    ctx.response_tx.send(Response::CommandRequest).unwrap();

    Ok(())
}

// This function is called when the velocity is zero.
// Move forward a [distance] millimeter and exits without decelerating.
pub(super) fn start(ctx: &mut ControlContext, distance: f32) -> anyhow::Result<()> {
    ctx.theta_pid = pid::Pid::new(ctx.config.search_ctrl_cfg.theta_pid.clone());
    ctx.omega_pid = pid::Pid::new(ctx.config.search_ctrl_cfg.omega_pid.clone());
    ctx.v_pid = pid::Pid::new(ctx.config.search_ctrl_cfg.v_pid.clone());
    ctx.pos_pid = pid::Pid::new(ctx.config.search_ctrl_cfg.pos_pid.clone());

    control_thread::reset_micromouse_state(ctx);
    // uprintln!("Start straight:\n  distance={}mm", distance);
    // uprintln!("  straight={:?}", straight);
    motor::set_l(0.0);
    motor::set_r(0.0);
    motor::enable(true);

    // uprintln!("start log");
    ctx.start_log(0);

    // accelerate
    go(
        ctx,
        distance,
        ctx.config.search_ctrl_cfg.vel_fwd,
        Some(0.07 - 0.027),
    )?;

    ctx.response_tx.send(Response::CommandRequest).unwrap();

    Ok(())
}

pub(super) fn forward(ctx: &mut ControlContext, distance: f32) -> anyhow::Result<()> {
    // constant speed
    go(
        ctx,
        distance,
        ctx.config.search_ctrl_cfg.vel_fwd,
        Some(0.07),
    )
}

pub(super) fn test(ctx: &mut ControlContext) -> anyhow::Result<()> {
    let target = mm_traj::State::default();
    motor::set_l(10.0);
    motor::set_r(10.0);
    motor::enable(true);
    ctx.start_log(0);
    for _ in 0..2000 {
        control_thread::measure(ctx)?;
        let _ = control_thread::update(ctx);
        control_thread::update_target(ctx, &target);
        ctx.log();
        timer_interrupt::sync_ms();
    }
    ctx.stop_log();
    motor::set_l(0.0);
    motor::set_r(0.0);
    motor::enable(false);
    ctx.response_tx.send(Response::CommandRequest).unwrap();
    Ok(())
}
