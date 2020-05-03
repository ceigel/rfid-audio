use hal::gpio::{gpioe, Output, PushPull};
use hal::prelude::*;
use stm32f3xx_hal as hal;

pub struct Leds {
    led_red: hal::gpio::PXx<Output<PushPull>>,
    led_orange: hal::gpio::PXx<Output<PushPull>>,
    led_green: hal::gpio::PXx<Output<PushPull>>,
}

pub enum LedsState {
    Green,
    Yellow,
    Red,
}

impl Leds {
    pub fn new(mut gpioe: gpioe::Parts) -> Self {
        Self {
            led_red: gpioe
                .pe9
                .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
                .downgrade()
                .downgrade(),
            led_orange: gpioe
                .pe10
                .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
                .downgrade()
                .downgrade(),
            led_green: gpioe
                .pe11
                .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
                .downgrade()
                .downgrade(),
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
