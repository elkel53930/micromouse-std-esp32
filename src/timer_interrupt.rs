use crate::context;
use crate::encoder;
use crate::imu;
use crate::wall_sensor;

#[derive(Debug, Clone, Copy, PartialEq)]
enum InterruptSequence {
    ReadBattEnableLs,
    ReadLsEnableLf,
    ReadLfEnableRf,
    ReadRfEnableRs,
    ReadRsDisable,
    ReadImu,
    ReadEncoders,
    None,
}

const SEQUENCE: [InterruptSequence; 10] = [
    InterruptSequence::ReadBattEnableLs,
    InterruptSequence::ReadLsEnableLf,
    InterruptSequence::ReadLfEnableRf,
    InterruptSequence::ReadRfEnableRs,
    InterruptSequence::ReadRsDisable,
    InterruptSequence::ReadImu,
    InterruptSequence::ReadEncoders,
    InterruptSequence::None,
    InterruptSequence::None,
    InterruptSequence::None,
];

pub fn interrupt() -> anyhow::Result<()> {
    let mut ctx = context::get();
    let step = SEQUENCE[ctx.step as usize];
    ctx.step = (ctx.step + 1) % SEQUENCE.len() as u8;

    match step {
        InterruptSequence::ReadBattEnableLs => {
            ctx.batt_raw = wall_sensor::read_batt()?;
            if ctx.enable_ls {
                wall_sensor::on_ls()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadLsEnableLf => {
            if ctx.enable_ls {
                ctx.ls_raw = wall_sensor::read_ls()?;
            }

            if ctx.enable_lf {
                wall_sensor::on_lf()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadLfEnableRf => {
            if ctx.enable_lf {
                ctx.lf_raw = wall_sensor::read_lf()?;
            }

            if ctx.enable_rf {
                wall_sensor::on_rf()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadRfEnableRs => {
            if ctx.enable_rf {
                ctx.rf_raw = wall_sensor::read_rf()?;
            }

            if ctx.enable_rs {
                wall_sensor::on_rs()?;
            } else {
                wall_sensor::off()?;
            }
        }

        InterruptSequence::ReadRsDisable => {
            if ctx.enable_rs {
                ctx.rs_raw = wall_sensor::read_rs()?;
            }
            wall_sensor::off()?;
        }

        InterruptSequence::ReadImu => {
            imu::read()?;
        }

        InterruptSequence::ReadEncoders => {
            ctx.enc_l_raw = encoder::read_l()?;
            ctx.enc_r_raw = encoder::read_r()?;
        }

        InterruptSequence::None => {}
    }
    context::set(&ctx);
    Ok(())
}
