use crate::hal;
use crate::hal::hal as embedded_hal;
use embedded_hal::digital::v2::OutputPin;
use hal::gpio::{gpioa, Output, PushPull};
use hal::prelude::*;
use hal::pwm::{Pwm, C1, C2, C3, C4};
use hal::stm32::TIM2;
use log::error;

#[derive(Debug, PartialEq)]
#[allow(unused)]
pub enum InitState {
    Begin,
    SetSDCard,
    SetRFID,
    InitADC,
    InitFinished,
}

pub enum State {
    Init(InitState),
}

#[derive(Debug)]
pub enum Error {
    GpioError,
}

pub struct StateLeds {
    state: State,
    nose_b: Pwm<TIM2, C1>,
    nose_g: Pwm<TIM2, C2>,
    nose_r: Pwm<TIM2, C3>,
    eye: gpioa::PA11<Output<PushPull>>,
    mouth: Pwm<TIM2, C4>,
}

impl StateLeds {
    pub fn new(
        nose_b: Pwm<TIM2, C1>,
        nose_g: Pwm<TIM2, C2>,
        nose_r: Pwm<TIM2, C3>,
        eye: gpioa::PA11<Output<PushPull>>,
        mouth: Pwm<TIM2, C4>,
    ) -> Self {
        let mut s = Self {
            state: State::Init(InitState::Begin),
            nose_b,
            nose_g,
            nose_r,
            eye,
            mouth,
        };
        s.init_leds();
        s
    }

    pub fn set_init_state(&mut self, init_state: InitState) {
        self.state = State::Init(init_state);
        if let Err(Error::GpioError) = self.show_state() {
            error!("Can't set leds");
        }
    }

    pub fn show_state(&mut self) -> Result<(), self::Error> {
        self.reset();
        match &self.state {
            State::Init(init_state) => match init_state {
                InitState::Begin => self.eye.set_high().map_err(|_| self::Error::GpioError)?,
                InitState::SetSDCard => self.nose_b.set_duty(0),
                InitState::SetRFID => self.nose_r.set_duty(0),
                InitState::InitADC => self.nose_g.set_duty(0),
                InitState::InitFinished => {
                    self.mouth.set_duty(self.mouth.get_max_duty());
                    self.nose_g.set_duty(0);
                }
            },
        }

        Ok(())
    }
    fn init_leds(&mut self) {
        self.eye.set_low().ok();
        self.nose_r.enable();
        self.nose_g.enable();
        self.nose_b.enable();
        self.mouth.enable();
        self.reset();
    }

    fn reset(&mut self) {
        self.nose_r.set_duty(self.nose_r.get_max_duty());
        self.nose_g.set_duty(self.nose_g.get_max_duty());
        self.nose_b.set_duty(self.nose_b.get_max_duty());
        self.mouth.set_duty(0);
    }
}
