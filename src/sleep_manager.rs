use crate::state::{State, StateLeds};
use core::time::Duration;
use cortex_m;

pub struct SleepManager {
    current_click_count: u32,
    sleep_click_count: u32,
}

impl SleepManager {
    pub fn new(sleep_time: Duration, click_freq: Duration) -> Self {
        Self {
            current_click_count: 0,
            sleep_click_count: (sleep_time.as_millis() / click_freq.as_millis()) as u32,
        }
    }

    pub fn click(&mut self, reset_sleep: bool, state_leds: &mut StateLeds) {
        if reset_sleep {
            self.current_click_count = 0;
        } else {
            self.current_click_count += 1;
        }
        if self.current_click_count >= self.sleep_click_count {
            state_leds.set_state(State::ShutDown);
            cortex_m::asm::wfi();
        }
    }
}
