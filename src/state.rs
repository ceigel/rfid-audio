use crate::hal;
use crate::hal::hal as embedded_hal;
use embedded_hal::digital::v2::OutputPin;
use hal::gpio::{gpioa, gpiob, Output, PushPull};
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
    nose_b: gpioa::PA0<Output<PushPull>>,
    nose_g: gpioa::PA1<Output<PushPull>>,
    nose_r: gpioa::PA2<Output<PushPull>>,
    eye: gpioa::PA11<Output<PushPull>>,
    mouth: gpiob::PB11<Output<PushPull>>,
}

impl StateLeds {
    pub fn new(
        nose_b: gpioa::PA0<Output<PushPull>>,
        nose_g: gpioa::PA1<Output<PushPull>>,
        nose_r: gpioa::PA2<Output<PushPull>>,
        eye: gpioa::PA11<Output<PushPull>>,
        mouth: gpiob::PB11<Output<PushPull>>,
    ) -> Self {
        let mut s = Self {
            state: State::Init(InitState::Begin),
            nose_b,
            nose_g,
            nose_r,
            eye,
            mouth,
        };
        s.turn_off();
        s
    }

    pub fn set_init_state(&mut self, init_state: InitState) {
        self.state = State::Init(init_state);
        if let Err(Error::GpioError) = self.show_state() {
            error!("Can't set leds");
        }
    }

    pub fn show_state(&mut self) -> Result<(), self::Error> {
        match &self.state {
            State::Init(init_state) => match init_state {
                InitState::Begin => self.eye.set_high(),
                InitState::SetSDCard => self.nose_b.set_low(),
                InitState::SetRFID => self.nose_b.set_high().and_then(|_| self.nose_r.set_low()),
                InitState::InitADC => self.nose_r.set_high().and_then(|_| self.nose_g.set_low()),
                InitState::InitFinished => self.mouth.set_high(),
            },
        }
        .map_err(|_| self::Error::GpioError)?;
        Ok(())
    }
    fn turn_off(&mut self) {
        self.nose_b
            .set_high()
            .and_then(|_| self.nose_g.set_high())
            .and_then(|_| self.nose_r.set_high())
            .and_then(|_| self.eye.set_low())
            .and_then(|_| self.mouth.set_low())
            .ok();
    }
}
