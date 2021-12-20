use crate::hal::gpio::{gpiob, Floating, Input, PullUp, MODER, PUPDR};
pub enum ButtonKind {
    Previous,
    Pause,
    Next,
}

pub struct Buttons {
    pub button_next: gpiob::PB2<Input<PullUp>>,
    pub button_prev: gpiob::PB12<Input<PullUp>>,
    pub button_pause: gpiob::PB10<Input<PullUp>>,
}

impl Buttons {
    pub fn new(
        prev: gpiob::PB12<Input<Floating>>,
        pause: gpiob::PB10<Input<Floating>>,
        next: gpiob::PB2<Input<Floating>>,
        moder: &mut MODER<'B'>,
        pupdr: &mut PUPDR<'B'>,
    ) -> Self {
        let button_next = next.into_pull_up_input(moder, pupdr);
        let button_prev = prev.into_pull_up_input(moder, pupdr);
        let button_pause = pause.into_pull_up_input(moder, pupdr);

        Buttons {
            button_next,
            button_prev,
            button_pause,
        }
    }
}
