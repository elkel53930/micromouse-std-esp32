use crate::led::LedColor::*;
use crate::OperationContext;
use esp_idf_hal::delay::FreeRtos;

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum UserOperation {
    HoldL,
    HoldR,
}

pub fn hold_ws(ctx: &OperationContext) -> UserOperation {
    ctx.led_tx.send((Blue, Some("01"))).unwrap();
    let result;
    loop {
        let (rs, ls) = {
            let wall_sensor = ctx.ods.wall_sensor.lock().unwrap();
            (
                wall_sensor.rs_raw.unwrap_or(0),
                wall_sensor.ls_raw.unwrap_or(0),
            )
        };
        if ls > 1200 {
            result = UserOperation::HoldL;
            break;
        }
        if rs > 1200 {
            result = UserOperation::HoldR;
            break;
        }
    }
    ctx.led_tx.send((Blue, None)).unwrap();
    result
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
