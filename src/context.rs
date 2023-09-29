use esp_idf_hal::task::CriticalSectionGuard;

#[derive(Debug, Clone, Copy)]
pub struct Context {
    pub step: u8,
    pub enc_l_raw: u16,
    pub enc_r_raw: u16,
}

static mut CONTEXT: Context = Context {
    step: 0,
    enc_l_raw: 0,
    enc_r_raw: 0,
};

pub fn ope<F>(_cs: &CriticalSectionGuard<'_>, mut closure: F)
where
    F: FnMut(&mut Context),
{
    unsafe {
        closure(&mut CONTEXT);
    }
}

pub fn ope_r<F>(_cs: &CriticalSectionGuard<'_>, mut closure: F) -> anyhow::Result<()>
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

pub fn enter_ics() -> InterruptCriticalSectionGuard {
    InterruptCriticalSectionGuard::new()
}

pub struct InterruptCriticalSectionGuard {}

impl Drop for InterruptCriticalSectionGuard {
    fn drop(&mut self) {
        unsafe {
            esp_idf_sys::esp_intr_noniram_enable();
        }
    }
}

impl InterruptCriticalSectionGuard {
    pub fn new() -> Self {
        unsafe {
            esp_idf_sys::esp_intr_noniram_disable();
        }
        Self {}
    }
}

pub enum WriteByInterrupt<T> {
    PreInit,
    Data(T),
}

impl<T> WriteByInterrupt<T> {
    pub fn write<F>(&mut self, mut f: F)
    where
        F: FnMut(&mut T),
    {
        if let WriteByInterrupt::Data(ref mut data) = self {
            f(data);
        } else {
            panic!("WriteByInterrupt::write: called on WriteByInterrupt::PreInit");
        }
    }

    pub fn read<F>(&self, _: &InterruptCriticalSectionGuard, f: F)
    where
        F: FnOnce(&T),
    {
        if let WriteByInterrupt::Data(ref data) = self {
            f(data);
        } else {
            panic!("WriteByInterrupt::read: called on WriteByInterrupt::PreInit");
        }
    }

    pub fn get(&self) -> &T {
        if let WriteByInterrupt::Data(ref data) = self {
            data
        } else {
            panic!("WriteByInterrupt::get: called on WriteByInterrupt::PreInit");
        }
    }
}

pub enum ShareWithThread<T> {
    PreInit,
    Data(T),
}

impl<T> ShareWithThread<T> {
    pub fn access<F>(&mut self, _: &CriticalSectionGuard<'_>, mut f: F)
    where
        F: FnMut(&mut T),
    {
        if let ShareWithThread::Data(ref mut data) = self {
            f(data);
        } else {
            panic!("ShareWithThread::access: called on ShareWithThread::PreInit");
        }
    }

    pub fn get(&self) -> &T {
        if let ShareWithThread::Data(ref data) = self {
            data
        } else {
            panic!("ShareWithThread::get: called on ShareWithThread::PreInit");
        }
    }
}
