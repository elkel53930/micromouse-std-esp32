use crate::control_thread::{self, ControlContext, Response};
use crate::motor;
use crate::ods::MicromouseState;
use crate::pid;
use crate::timer_interrupt;

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

// This function is called when the velocity is zero.
// Move forward a [distance] millimeter and exits without decelerating.
pub(super) fn start(ctx: &mut ControlContext, distance: f32) -> anyhow::Result<()> {
    let mut theta_pid = pid::Pid::new(ctx.config.search_ctrl_cfg.theta_pid.clone());
    let mut omega_pid = pid::Pid::new(ctx.config.search_ctrl_cfg.omega_pid.clone());
    let target_theta = std::f32::consts::PI / 2.0;
    let target_omega = 0.0;

    motor::enable(true);
    control_thread::reset_micromouse_state(ctx);
    let target_v = ctx.config.search_ctrl_cfg.vel_fwd;

    ctx.start_log(0);
    control_thread::set_motor_duty(ctx, 100.0, 100.0);
    for _ in 0..2 {
        control_thread::measure(ctx)?;
        control_thread::update(ctx);
        ctx.log();
        timer_interrupt::sync_ms();
    }

    for _ in 0..200 {
        crate::led::on(crate::led::LedColor::Red)?;
        control_thread::measure(ctx)?;
        let micromouse = control_thread::update(ctx);
        let fb_theta = theta_pid.update(micromouse.theta - target_theta);
        let fb_omega = omega_pid.update(micromouse.omega - target_omega);
        let volt_r = calc_ff_r(ctx, target_v);
        let volt_l = calc_ff_l(ctx, target_v);
        let duty_r = calc_duty(&micromouse, volt_r + fb_theta + fb_omega);
        let duty_l = calc_duty(&micromouse, volt_l - fb_theta - fb_omega);
        control_thread::set_motor_duty(ctx, duty_l, duty_r);
        ctx.log();
        crate::led::off(crate::led::LedColor::Red)?;
        timer_interrupt::sync_ms();
    }
    motor::set_l(0.0);
    motor::set_r(0.0);
    ctx.stop_log();
    ctx.response_tx.send(Response::CommandRequest).unwrap();
    Ok(())
}

pub(super) fn stop(ctx: &mut ControlContext) -> anyhow::Result<()> {
    panic!("Not implemented yet");
}
