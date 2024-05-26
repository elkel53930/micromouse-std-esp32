use crate::control_thread::{self, ControlContext, Response};
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
    let target_v = 0.5; //ctx.config.search_ctrl_cfg.vel_fwd;

    ctx.start_log(1);
    control_thread::set_motor_duty(ctx, 100.0, 100.0);
    for _ in 0..2 {
        control_thread::measure(ctx)?;
        control_thread::update(ctx);
        ctx.log();
        timer_interrupt::sync_ms();
    }

    let mut calc_volt_duty: u32 = 0;
    let mut set_duty: u32 = 0;
    let mut call_measure: u32 = 0;
    let mut call_update: u32 = 0;
    let mut call_log: u32 = 0;

    let time = timer_interrupt::get_ms();
    for _ in 0..100 {
        crate::led::on(crate::led::LedColor::Red)?;
        let volt_r = calc_ff_r(ctx, target_v);
        let volt_l = calc_ff_l(ctx, target_v);
        let duty_r = calc_duty(ctx, volt_r);
        let duty_l = calc_duty(ctx, volt_l);
        calc_volt_duty += timer_interrupt::get_us();
        control_thread::set_motor_duty(ctx, duty_l, duty_r);
        set_duty += timer_interrupt::get_us();
        control_thread::measure(ctx)?;
        call_measure += timer_interrupt::get_us();
        control_thread::update(ctx);
        call_update += timer_interrupt::get_us();
        ctx.log();
        call_log += timer_interrupt::get_us();
        crate::led::off(crate::led::LedColor::Red)?;
        timer_interrupt::sync_ms();
    }
    let elapsed = timer_interrupt::get_ms() - time;
    uprintln!("Elapsed: {} ms", elapsed);
    uprintln!("calc_volt_duty: {} us", calc_volt_duty);
    uprintln!("set_duty: {} us", set_duty);
    uprintln!("call_measure: {} us", call_measure);
    uprintln!("call_update: {} us", call_update);
    uprintln!("call_log: {} us", call_log);
    motor::set_l(0.0);
    motor::set_r(0.0);
    ctx.stop_log();
    ctx.response_tx.send(Response::CommandRequest).unwrap();
    Ok(())
}

pub(super) fn stop(ctx: &mut ControlContext) -> anyhow::Result<()> {
    panic!("Not implemented yet");
}
