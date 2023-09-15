use crate::control::ControlContext;

#[derive(Debug, Clone, Copy)]
pub struct Context {
    pub step: u8,
    pub enable_ls: bool,
    pub enable_lf: bool,
    pub enable_rf: bool,
    pub enable_rs: bool,

    pub batt_raw: u16,
    pub ls_raw: u16,
    pub lf_raw: u16,
    pub rf_raw: u16,
    pub rs_raw: u16,
    pub gyro_yaw_raw: i16,
    pub enc_l_raw: u16,
    pub enc_r_raw: u16,

    pub control_context: ControlContext,
}

static mut CONTEXT: Context = Context {
    step: 0,
    enable_ls: false,
    enable_lf: false,
    enable_rf: false,
    enable_rs: false,
    batt_raw: 0,
    ls_raw: 0,
    lf_raw: 0,
    rf_raw: 0,
    rs_raw: 0,
    gyro_yaw_raw: 0,
    enc_l_raw: 0,
    enc_r_raw: 0,
    control_context: ControlContext {
        gyro_yaw: 0.0,
        ls: 0.0,
        lf: 0.0,
        rf: 0.0,
        rs: 0.0,
        enc_l: 0.0,
        enc_r: 0.0,
        batt: 0.0,
    },
};

pub fn ope<F>(mut closure: F)
where
    F: FnMut(&mut Context),
{
    unsafe {
        closure(&mut CONTEXT);
    }
}

pub fn ope_r<F>(mut closure: F) -> anyhow::Result<()>
where
    F: FnMut(&mut Context) -> anyhow::Result<()>,
{
    unsafe {
        closure(&mut CONTEXT)?;
    }
    Ok(())
}

pub fn get() -> Context {
    unsafe { CONTEXT }
}

pub fn set(ctx: &Context) {
    unsafe { CONTEXT = *ctx }
}
