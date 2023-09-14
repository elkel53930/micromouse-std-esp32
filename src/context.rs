#[derive(Debug, Clone, Copy, PartialEq)]

pub struct Context {
    pub step: u8,
    pub enable_ls: bool,
    pub enable_lf: bool,
    pub enable_rf: bool,
    pub enable_rs: bool,

    pub batt: u16,
    pub ls: u16,
    pub lf: u16,
    pub rf: u16,
    pub rs: u16,
    pub gyro: i16,
    pub enc_l: u16,
    pub enc_r: u16,
}

static mut CONTEXT: Context = Context {
    step: 0,
    enable_ls: false,
    enable_lf: false,
    enable_rf: false,
    enable_rs: false,
    batt: 0,
    ls: 0,
    lf: 0,
    rf: 0,
    rs: 0,
    gyro: 0,
    enc_l: 0,
    enc_r: 0,
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
