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
    ShutDown,
    Error,
}

#[derive(Debug)]
pub enum Error {
    GpioError,
}

struct PwmLed<P: PwmPin<Duty = u32>> {
    led: P,
    duty_idx: usize,
    light_level_increases: bool,
    revert: bool,
}

impl<P: PwmPin<Duty = u32>> PwmLed<P> {
    const VALUES_TABLE: [u32; 100] = [
        0, 2, 11, 25, 44, 69, 100, 137, 179, 228, 282, 343, 409, 483, 562, 648, 741, 841, 949,
        1063, 1185, 1315, 1453, 1599, 1754, 1917, 2089, 2271, 2462, 2663, 2874, 3096, 3328, 3571,
        3826, 4092, 4371, 4661, 4964, 5280, 5610, 5953, 6309, 6680, 7064, 7463, 7877, 8306, 8749,
        9208, 9681, 10170, 10674, 11193, 11727, 12275, 12838, 13415, 14006, 14610, 15227, 15857,
        16498, 17149, 17811, 18482, 19161, 19847, 20539, 21235, 21935, 22636, 23338, 24038, 24736,
        25429, 26115, 26794, 27462, 28118, 28761, 29388, 29997, 30587, 31155, 31700, 32220, 32712,
        33176, 33609, 34011, 34379, 34712, 35009, 35269, 35490, 35672, 35815, 35917, 35979,
    ];
    pub(crate) fn new(pwm: P, revert: bool) -> Self {
        Self {
            led: pwm,
            duty_idx: 0,
            light_level_increases: !revert,
            revert,
        }
    }

    pub(crate) fn enable(&mut self) {
        self.led.enable();
    }
    pub(crate) fn disable(&mut self) {
        self.led.disable();
    }

    pub(crate) fn set_duty(&mut self, duty: f32) {
        let duty = if self.revert { 1.0 - duty } else { duty };
        let current_duty = (duty * (self.led.get_max_duty() as f32)) as u32;
        self.led.set_duty(current_duty);
    }

    pub(crate) fn update_modulated(&mut self) {
        let current_idx = self.duty_idx;
        if self.light_level_increases && current_idx == Self::VALUES_TABLE.len() - 1 {
            self.light_level_increases = false;
        } else if !self.light_level_increases && current_idx == 0 {
            self.light_level_increases = true;
        }
        self.duty_idx = if self.light_level_increases {
            current_idx + 1
        } else {
            current_idx - 1
        };
        self.led.set_duty(Self::VALUES_TABLE[self.duty_idx]);
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
        self.display_count = self.display_count.wrapping_add(1);
        self.change_state_if_needed();
        if let Err(Error::GpioError) = self.display() {
            error!("Can't set leds");
        }
    }

    fn change_state_if_needed(&mut self) {
        match &self.state {
            State::PlaylistNotFound => self.set_state(State::NotPlaying),
            _ => {}
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
            State::ShutDown => {
                self.eye.set_low().ok();
                self.nose_r.disable();
                self.nose_g.disable();
                self.nose_b.disable();
                self.mouth.disable();
            }
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
