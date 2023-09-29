use crate::context;
use crate::encoder;
use crate::imu;
use crate::wall_sensor;

#[derive(Debug, Clone, Copy, PartialEq)]
enum InterruptSequence {
    ReadBattEnableLs,
    ReadLsEnableRs,
    ReadRsEnableLs,
    ReadLsEnableRf,
    ReadRfDisable,
    ReadImu,
    ReadEncoders,
    None,
}

const SEQUENCE: [InterruptSequence; 10] = [
    InterruptSequence::ReadBattEnableLs,
    InterruptSequence::ReadLsEnableRs,
    InterruptSequence::ReadRsEnableLs,
    InterruptSequence::ReadLsEnableRf,
    InterruptSequence::ReadRfDisable,
    InterruptSequence::ReadImu,
    InterruptSequence::ReadEncoders,
    InterruptSequence::None,
    InterruptSequence::None,
    InterruptSequence::None,
];

// Called from interrupt handler
pub fn interrupt() -> anyhow::Result<()> {
    let mut ctx = context::get();
    let step = SEQUENCE[ctx.step as usize];
    ctx.step = (ctx.step + 1) % SEQUENCE.len() as u8;

    match step {
        InterruptSequence::ReadBattEnableLs => {
            wall_sensor::sequence(wall_sensor::Sequence::ReadBattEnableLs)?;
        }

        InterruptSequence::ReadLsEnableRs => {
            wall_sensor::sequence(wall_sensor::Sequence::ReadLsEnableRs)?;
        }

        InterruptSequence::ReadRsEnableLs => {
            wall_sensor::sequence(wall_sensor::Sequence::ReadRsEnableLs)?;
        }

        InterruptSequence::ReadLsEnableRf => {
            wall_sensor::sequence(wall_sensor::Sequence::ReadLsEnableRf)?;
        }

        InterruptSequence::ReadRfDisable => {
            wall_sensor::sequence(wall_sensor::Sequence::ReadRfDisable)?;
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
