use crate::control_thread;
use crate::control_thread::{ControlContext, Response};
use crate::motor;
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

fn calc_duty(ctx: &ControlContext, voltage: f32) -> f32 {
    // battery voltage
    let vb = ctx.ods.wall_sensor.lock().unwrap().batt_phy;
    // voltage[V] -> duty[%]
    (voltage / vb) * 100.0
}

// This function is called when the velocity is zero.
// Move forward a [distance] millimeter and exits without decelerating.
pub(super) fn start(ctx: &mut ControlContext, distance: f32) -> anyhow::Result<()> {
    motor::enable(true);
    control_thread::reset_micromouse_state(ctx);
    let target_v = 1.0; // m/s
    let volt_r = calc_ff_r(ctx, target_v);
    let volt_l = calc_ff_l(ctx, target_v);
    let duty_r = calc_duty(ctx, volt_r);
    let duty_l = calc_duty(ctx, volt_l);
    uprintln!("SStart, duty_l: {}, duty_r: {}", duty_l, duty_r);
    motor::set_l(duty_l);
    motor::set_r(duty_r);
    let time = timer_interrupt::get_ms();
    for _ in 0..100 {
        control_thread::measure(ctx)?;
        timer_interrupt::sync_ms();
    }
    let elapsed = timer_interrupt::get_ms() - time;
    uprintln!("Elapsed: {} ms", elapsed);
    motor::set_l(0.0);
    motor::set_r(0.0);
    ctx.response_tx.send(Response::CommandRequest).unwrap();
    Ok(())
}

pub(super) fn stop(ctx: &mut ControlContext) -> anyhow::Result<()> {
    panic!("Not implemented yet");
    Ok(())
}
