use crate::state::{State, StateLeds};
use crate::RFIDReaderType;
use crate::SoundDevice;
use core::time::Duration;
use stm32l4xx_hal::stm32 as stm32l431;

pub struct SleepManager {
    current_click_count: u32,
    sleep_click_count: u32,
}

fn scr() -> &'static stm32l431::pwr::SCR {
    unsafe { &(*stm32l431::PWR::ptr()).scr }
}

impl SleepManager {
    pub fn new(sleep_time: Duration, click_freq: Duration) -> Self {
        Self {
            current_click_count: 0,
            sleep_click_count: (sleep_time.as_millis() / click_freq.as_millis()) as u32,
        }
    }

    pub fn shut_down(
        state_leds: &mut StateLeds,
        rfid_reader: &mut RFIDReaderType,
        sound_device: &mut SoundDevice,
    ) {
        state_leds.set_state(State::ShutDown);
        rfid_reader.sleep().ok();
        sound_device.stop_playing();
        scr().write(|w| {
            w.wuf1().set_bit();
            w.wuf2().set_bit();
            w.wuf3().set_bit();
            w.wuf4().set_bit();
            w.wuf5().set_bit();
            w
        });
        cortex_m::asm::wfi();
    }

    pub fn click(
        &mut self,
        reset_sleep: bool,
        state_leds: &mut StateLeds,
        rfid_reader: &mut RFIDReaderType,
        sound_device: &mut SoundDevice,
    ) {
        if reset_sleep {
            self.current_click_count = 0;
        } else {
            self.current_click_count += 1;
        }
        if self.current_click_count >= self.sleep_click_count {
            Self::shut_down(state_leds, rfid_reader, sound_device);
        }
    }
}
