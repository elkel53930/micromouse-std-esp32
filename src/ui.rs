use crate::control_thread;
use crate::led::LedColor::*;
use crate::OperationContext;
use esp_idf_hal::delay::FreeRtos;

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum UserOperation {
    TimeOut,
    HoldL,
    HoldR,
}

pub fn hold_ws(ctx: &OperationContext, timeout: Option<u16>) -> UserOperation {
    ctx.command_tx
        .send(control_thread::Command::SetActivateWallSensor(true))
        .unwrap();
    FreeRtos::delay_ms(10);
    ctx.led_tx.send((Blue, Some("01"))).unwrap();
    let mut result = UserOperation::TimeOut;
    let mut count = 0;
    loop {
        let (rs, ls) = {
            let wall_sensor = ctx.ods.lock().unwrap().wall_sensor;
            (wall_sensor.rs_raw.unwrap(), wall_sensor.ls_raw.unwrap())
        };
        if ls > 1200 {
            result = UserOperation::HoldL;
            break;
        }
        if rs > 1200 {
            result = UserOperation::HoldR;
            break;
        }
        if let Some(timeout) = timeout {
            count += 1;
            if count >= timeout {
                break;
            }
        }
        FreeRtos::delay_ms(10);
    }
    ctx.led_tx.send((Blue, Some("0"))).unwrap();
    ctx.command_tx
        .send(control_thread::Command::SetActivateWallSensor(false))
        .unwrap();

    log::info!("hold_ws: {:?}", result);
    result
}

pub fn wait(ctx: &OperationContext, expected: UserOperation) {
    ctx.led_tx.send((Green, Some("01"))).unwrap();
    assert!(expected == UserOperation::HoldL || expected == UserOperation::HoldR);
    loop {
        let actual = hold_ws(ctx, None);
        if actual == expected {
            break;
        }
    }
}

pub fn countdown(ctx: &OperationContext) {
    // Countdown
    ctx.led_tx.send((Red, Some("0"))).unwrap();
    ctx.led_tx.send((Blue, Some("0"))).unwrap();
    ctx.led_tx.send((Green, Some("1"))).unwrap();
    FreeRtos::delay_ms(1000);
    ctx.led_tx.send((Green, Some("0"))).unwrap();
    ctx.led_tx.send((Blue, Some("1"))).unwrap();
    FreeRtos::delay_ms(1000);
    ctx.led_tx.send((Blue, Some("0"))).unwrap();
    ctx.led_tx.send((Red, Some("1"))).unwrap();
    FreeRtos::delay_ms(1000);
    ctx.led_tx.send((Red, None)).unwrap();
    ctx.led_tx.send((Blue, None)).unwrap();
    ctx.led_tx.send((Green, None)).unwrap();
}
