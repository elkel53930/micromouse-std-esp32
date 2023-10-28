use crate::encoder;
use crate::imu;
use crate::led;
use crate::motor;
use crate::ods;
use crate::timer_interrupt::{sync_ms, wait_us};
use crate::wall_sensor;

use std::sync::Arc;

pub fn init(ods: &Arc<ods::Ods>) -> anyhow::Result<()> {
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: None,
        stack_size: 4096,
        priority: 15,
        inherit: false, // don't inherit this configuration across threads
        pin_to_core: Some(esp_idf_hal::cpu::Core::Core1),
    }
    .set()?;

    let ods = ods.clone();

    std::thread::Builder::new().spawn(move || -> anyhow::Result<()> {
        wall_sensor::off()?;
        loop {
            let batt = wall_sensor::read_batt()?;

            wall_sensor::on_ls()?;
            wait_us(30);
            let ls = wall_sensor::read_ls()?;

            wall_sensor::on_lf()?;
            wait_us(30);
            let lf = wall_sensor::read_lf()?;

            wall_sensor::on_rf()?;
            wait_us(30);
            let rf = wall_sensor::read_rf()?;

            wall_sensor::on_rs()?;
            wait_us(30);
            let rs = wall_sensor::read_rs()?;

            let gyro_x = imu::read()?;

            let encoder_l = encoder::read_l()?;
            let encoder_r = encoder::read_r()?;

            {
                let mut imu = ods.imu.lock().unwrap();
                (*imu).gyro_x_raw = gyro_x;
            }
            {
                let mut enc = ods.encoder.lock().unwrap();
                (*enc).l_raw = encoder_l;
                (*enc).r_raw = encoder_r;
            }
            {
                let mut ws = ods.wall_sensor.lock().unwrap();
                (*ws).ls_raw = ls;
                (*ws).lf_raw = lf;
                (*ws).rf_raw = rf;
                (*ws).rs_raw = rs;
                (*ws).batt_raw = batt;
            }

            sync_ms();
        }
    })?;

    Ok(())
}
