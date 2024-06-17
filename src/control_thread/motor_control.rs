use crate::control_thread::{self, ControlContext, Response};
use crate::motor;
use crate::ods::MicromouseState;
use crate::pid;
use crate::timer_interrupt;
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

fn calc_duty(micromouse: &MicromouseState, voltage: f32) -> f32 {
    // battery voltage
    let vb = micromouse.v_batt;
    // voltage[V] -> duty[%]
    (voltage / vb) * 100.0
}

fn calc_diff(xa: f32, ya: f32, xb: f32, yb: f32, theta: f32) -> f32 {
    let delta_x = xb - xa;
    let delta_y = yb - ya;
    let diff = delta_x * theta.cos() + delta_y * theta.sin();
    diff
}

fn go(ctx: &mut ControlContext, distance: f32, final_velocity: f32) -> anyhow::Result<()> {
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
    while flag {
        let target = straight.proceed();
        // uprintln!("target={:?}", target);
        let target = match target {
            TrajResult::Done(target) => {
                flag = false;
                target
            }
            TrajResult::Continue(target) => target,
        };

        crate::led::on(crate::led::LedColor::Red)?;
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);
        control_thread::update_target(ctx, &target);
        let fb_theta = ctx.theta_pid.update(target.theta - micromouse.theta);
        let fb_omega = ctx.omega_pid.update(target.omega - micromouse.omega);
        let fb_v = ctx.v_pid.update(target.v - micromouse.v);
        let diff_pos = calc_diff(micromouse.x, micromouse.y, target.x, target.y, target.theta);
        let diff_pos = ctx.pos_ave.update(diff_pos);
        let fb_pos = ctx.pos_pid.update(diff_pos);
        let ff_r = calc_ff_r(ctx, target.v);
        let ff_l = calc_ff_l(ctx, target.v);
        let duty_r = calc_duty(&micromouse, ff_r + fb_pos + fb_v + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, ff_l + fb_pos + fb_v - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);
        ctx.log();
        crate::led::off(crate::led::LedColor::Red)?;
        timer_interrupt::sync_ms();
    }
    Ok(())
}

fn stop(ctx: &mut ControlContext, duration: usize) -> anyhow::Result<()> {
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

    for _ in 0..duration {
        crate::led::on(crate::led::LedColor::Red)?;
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);
        control_thread::update_target(ctx, &target);
        let fb_theta = ctx.theta_pid.update(target.theta - micromouse.theta);
        let fb_omega = ctx.omega_pid.update(target.omega - micromouse.omega);
        let fb_v = ctx.v_pid.update(target.v - micromouse.v);
        let diff_pos = calc_diff(micromouse.x, micromouse.y, target.x, target.y, target.theta);
        let diff_pos = ctx.pos_ave.update(diff_pos);
        let fb_pos = ctx.pos_pid.update(diff_pos);
        let ff_r = calc_ff_r(ctx, target.v);
        let ff_l = calc_ff_l(ctx, target.v);
        let duty_r = calc_duty(&micromouse, ff_r + fb_pos + fb_v + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, ff_l + fb_pos + fb_v - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);
        ctx.log();
        crate::led::off(crate::led::LedColor::Red)?;
        timer_interrupt::sync_ms();
    }
    control_thread::set_motor_duty(ctx, 0.0, 0.0);
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
    go(ctx, 0.09 - 0.027, ctx.config.search_ctrl_cfg.vel_fwd)?;

    // constant speed
    go(ctx, 0.09, ctx.config.search_ctrl_cfg.vel_fwd)?;

    // decelerate
    go(ctx, 0.045, 0.0)?;

    stop(ctx, 100)?;
    ctx.stop_log();

    ctx.response_tx.send(Response::CommandRequest).unwrap();

    Ok(())
}
