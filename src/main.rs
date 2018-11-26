#![no_std]
#![no_main]

extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger
extern crate stm32f30x;

use cortex_m_semihosting::{debug, hprintln};
use stm32f30x::{interrupt, Interrupt};
use rtfm::app;

#[app(device = stm32f30x)]
const APP: () = {
    #[init]
    fn init() {
        rtfm::pend(Interrupt::SPI1);
        hprintln!("init").unwrap();
    }

    #[idle]
    fn idle() -> ! {
        hprintln!("idle").unwrap();

        rtfm::pend(Interrupt::SPI1);

        hprintln!("idle 2").unwrap();

        loop {}
    }

    #[interrupt]
    fn SPI1() {
        static mut TIMES: u32 = 0;

        // Safe access to local `static mut` variable
        *TIMES += 1;

        hprintln!(
            "SPI1 called {} time{}",
            *TIMES,
            if *TIMES > 1 { "s" } else { "" }
        ).unwrap();
    }
};

