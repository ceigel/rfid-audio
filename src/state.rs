use crate::hal;
use crate::hal::hal as embedded_hal;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use hal::gpio::{gpioa, Output, PushPull};
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
    Playing,
    NotPlaying,
    PlaylistNotFound,
    Error,
}

#[derive(Debug)]
pub enum Error {
    GpioError,
}

struct PwmLed<P: PwmPin<Duty = u32>> {
    led: P,
    current_duty: u32,
    min_duty: u32,
    max_duty: u32,
    light_level_increases: bool,
    revert: bool,
}

impl<P: PwmPin<Duty = u32>> PwmLed<P> {
    pub(crate) fn new(pwm: P, revert: bool) -> Self {
        let min_duty = 0;
        let max_duty = pwm.get_max_duty();
        Self {
            led: pwm,
            current_duty: min_duty,
            min_duty,
            max_duty,
            light_level_increases: !revert,
            revert,
        }
    }

    pub(crate) fn enable(&mut self) {
        self.led.enable();
    }

    pub(crate) fn set_duty(&mut self, duty: f32) {
        let duty = if self.revert { 1.0 - duty } else { duty };
        self.current_duty = (duty * (self.max_duty as f32)) as u32;
        self.led.set_duty(self.current_duty);
    }

    pub(crate) fn update_modulated(&mut self) {
        const MODULATION_STEP_SIZE: u32 = 6000;
        let max_duty = self.max_duty;
        let min_duty = self.min_duty;
        let current_duty = self.current_duty;
        let increment = MODULATION_STEP_SIZE;
        self.current_duty = if self.light_level_increases && max_duty - current_duty < increment {
            self.light_level_increases = false;
            max_duty
        } else if !self.light_level_increases && current_duty - min_duty < increment {
            self.light_level_increases = true;
            self.min_duty
        } else {
            if self.light_level_increases {
                current_duty + increment
            } else {
                current_duty - increment
            }
        };
        self.led.set_duty(self.current_duty);
    }
}
pub struct StateLeds {
    state: State,
    nose_b: PwmLed<Pwm<TIM2, C1>>,
    nose_g: PwmLed<Pwm<TIM2, C2>>,
    nose_r: PwmLed<Pwm<TIM2, C3>>,
    eye: gpioa::PA11<Output<PushPull>>,
    mouth: PwmLed<Pwm<TIM2, C4>>,
    display_count: u32,
}

impl StateLeds {
    const DISPLAY_CYCLE: u32 = 1;
    pub fn new(
        nose_b: Pwm<TIM2, C1>,
        nose_g: Pwm<TIM2, C2>,
        nose_r: Pwm<TIM2, C3>,
        eye: gpioa::PA11<Output<PushPull>>,
        mouth: Pwm<TIM2, C4>,
    ) -> Self {
        let mut s = Self {
            state: State::Init(InitState::Begin),
            nose_b: PwmLed::new(nose_b, true),
            nose_g: PwmLed::new(nose_g, true),
            nose_r: PwmLed::new(nose_r, true),
            eye,
            mouth: PwmLed::new(mouth, false),
            display_count: 0u32,
        };
        s.init_leds();
        s
    }

    pub fn set_state(&mut self, state: State) {
        self.state = state;
        self.display_count = 0;
        if let Err(Error::GpioError) = self.update_state() {
            error!("Can't set leds");
        }
    }

    pub fn show_state(&mut self) {
        self.display_count += 1;
        if self.display_count == Self::DISPLAY_CYCLE {
            self.display_count = 0
        }
        if self.display_count == 0 {
            if let Err(Error::GpioError) = self.display() {
                error!("Can't set leds");
            }
        }
    }

    fn display(&mut self) -> Result<(), self::Error> {
        match &self.state {
            State::Error => {
                self.nose_r.update_modulated();
            }
            State::Playing => self.nose_g.update_modulated(),
            State::NotPlaying => self.nose_b.update_modulated(),
            State::PlaylistNotFound => self.nose_r.update_modulated(),
            _ => {}
        }

        Ok(())
    }

    fn update_state(&mut self) -> Result<(), self::Error> {
        self.reset();
        match &self.state {
            State::Init(init_state) => match init_state {
                InitState::Begin => self.eye.set_high().map_err(|_| self::Error::GpioError)?,
                InitState::SetSDCard => self.nose_b.set_duty(1.0),
                InitState::SetRFID => self.nose_r.set_duty(1.0),
                InitState::InitADC => self.nose_g.set_duty(1.0),
                InitState::InitFinished => {
                    self.mouth.set_duty(1.0);
                    self.nose_g.set_duty(1.0);
                }
            },
            State::Error => {
                self.mouth.set_duty(0.0);
                self.eye.set_low().ok();
            }
            State::Playing => {
                self.mouth.set_duty(1.0);
            }
            State::NotPlaying => {
                self.mouth.set_duty(1.0);
            }
            State::PlaylistNotFound => {
                self.mouth.set_duty(1.0);
            }
        }

        self.display()
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
        self.nose_r.set_duty(0.0);
        self.nose_g.set_duty(0.0);
        self.nose_b.set_duty(0.0);
        self.mouth.set_duty(0.0);
    }
}
