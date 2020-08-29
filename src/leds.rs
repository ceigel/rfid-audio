#![allow(dead_code)]
use hal::gpio::{gpioe, Output, PushPull};
use hal::prelude::*;
use stm32l4xx_hal as hal;

pub struct RealLeds {
    led_red: gpioe::PE9<Output<PushPull>>,
    led_orange: gpioe::PE10<Output<PushPull>>,
    led_green: gpioe::PE11<Output<PushPull>>,
}

pub enum LedsState {
    Green,
    Yellow,
    Red,
}

impl RealLeds {
    pub fn new(mut gpioe: gpioe::Parts) -> Self {
        Self {
            led_red: gpioe
                .pe9
                .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper),
            led_orange: gpioe
                .pe10
                .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper),
            led_green: gpioe
                .pe11
                .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper),
        }
    }
    pub fn set_state(&mut self, state: LedsState, clear: bool) {
        if clear {
            self.clear_state();
        }
        let res = match state {
            LedsState::Red => self.led_red.set_high(),
            LedsState::Yellow => self.led_orange.set_high(),
            LedsState::Green => self.led_green.set_high(),
        };
        res.expect("To set status leds");
    }

    pub fn clear_state(&mut self) {
        self.led_red.set_low().expect("To reset red led");
        self.led_orange.set_low().expect("To reset red led");
        self.led_green.set_low().expect("To reset red led");
    }
}

pub struct FakeLeds();
impl FakeLeds {
    pub fn new(mut _gpioe: gpioe::Parts) -> Self {
        Self {}
    }
    pub fn set_state(&mut self, _state: LedsState, _clear: bool) {}
    pub fn clear_state(&mut self) {}
}

pub type Leds = RealLeds;
