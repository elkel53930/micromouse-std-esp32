use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::timer;

static mut COUNTER_MS: u32 = 0;
static mut TIMER: Option<timer::TimerDriver> = None;

pub fn init(peripherals: &mut Peripherals) -> anyhow::Result<()> {
    unsafe {
        COUNTER_MS = 0;
    }
    let timer_config = timer::TimerConfig::new().auto_reload(true);
    let timer00 = unsafe { peripherals.timer00.clone_unchecked() };
    let mut timer = timer::TimerDriver::new(timer00, &timer_config)?;
    timer.set_alarm(1000)?;
    unsafe {
        timer.subscribe(timer_isr)?;
    }
    timer.enable_alarm(true)?;
    timer.enable(true)?;
    unsafe {
        TIMER = Some(timer);
    }

    Ok(())
}

// Get the current time in microseconds
// It is reset to 0 every 1000us
fn get_us() -> u32 {
    // The timer counter is incremented by hardware every 1us.
    let counter = unsafe { TIMER.as_mut().unwrap().counter().unwrap() };
    counter as u32
}

// Wait for a given number of microseconds
pub fn wait_us(duration_us: u32) {
    if duration_us > 999 {
        panic!("duration_us exceeds maximum allowable value of 999");
    }
    let start_us = get_us();
    let mut current_us = start_us;

    loop {
        let elapsed_us = if current_us >= start_us {
            current_us - start_us
        } else {
            (1000 - start_us) + current_us
        };

        if elapsed_us >= duration_us {
            break;
        }

        current_us = get_us();
    }
}

// Wait until the timer counter is reset to 0
pub fn sync_ms() {
    let mut prev = get_us();
    loop {
        let now = get_us();
        if prev > 500 && now < 500 {
            break;
        }
        prev = now;
    }
}

// Timer interrupt handler
fn timer_isr() {
    unsafe {
        COUNTER_MS += 1;
    }
}

// Get the time in milliseconds since the timer was started
pub fn get_ms() -> u32 {
    unsafe { COUNTER_MS }
}
