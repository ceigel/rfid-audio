#![allow(dead_code)]
use cortex_m::asm;
use stm32l4xx_hal::gpio::{gpiob, Analog, Floating, Input};
use stm32l4xx_hal::pac::{self, rcc, ADC};

const MAX_ADVREGEN_STARTUP_US: u32 = 10;
pub fn ahb2enr() -> &'static rcc::AHB2ENR {
    unsafe { &(*pac::RCC::ptr()).ahb2enr }
}

fn wait_advregen_startup(clocks: &stm32l4xx_hal::rcc::Clocks) {
    asm::delay((MAX_ADVREGEN_STARTUP_US * 1_000_000) / clocks.sysclk().0);
}

#[allow(dead_code)]
pub struct BatteryReader {
    pin: gpiob::PB0<Analog>,
    adc: ADC,
}

impl BatteryReader {
    pub fn new(
        pb0: gpiob::PB0<Input<Floating>>,
        adc: ADC,
        moder: &mut gpiob::MODER,
        pupdr: &mut gpiob::PUPDR,
        clocks: &stm32l4xx_hal::rcc::Clocks,
    ) -> Self {
        let pb0 = pb0.into_analog(moder, pupdr); // Speaker out
        ahb2enr().modify(|_, w| w.adcen().set_bit());
        adc.cr.write(|w| {
            w.deeppwd().clear_bit();
            w.advregen().set_bit();
            w
        });
        wait_advregen_startup(clocks);
        adc.cr.modify(|_, w| w.addis().set_bit());
        /*adc.cr
            .modify(|_, w| w.adcaldif().clear_bit().adcal().set_bit());

        while adc.cr.read().adcal().bit_is_set() {}*/
        BatteryReader { pin: pb0, adc }
    }

    pub fn read(&mut self) -> u16 {
        self.power_up();
        self.configure();

        while self.adc.isr.read().eos().bit_is_clear() {}

        let res = self.adc.dr.read().bits() as u16;
        self.power_down();
        res
    }

    pub fn sleep(&mut self) {
        self.adc.cr.modify(|_, w| w.deeppwd().set_bit());
    }

    fn power_up(&mut self) {
        self.adc.isr.modify(|_, w| w.adrdy().set_bit());
        self.adc.cr.modify(|_, w| {
            w.deeppwd().clear_bit();
            w.advregen().set_bit();
            w.aden().set_bit();
            w
        });
        while self.adc.isr.read().adrdy().bit_is_clear() {}
    }

    fn power_down(&mut self) {
        self.adc.cr.modify(|_, w| {
            w.advregen().clear_bit();
            w.addis().set_bit();
            w
        });
        self.adc.isr.modify(|_, w| w.adrdy().set_bit());
        while self.adc.cr.read().aden().bit_is_set() {}
    }

    fn configure(&mut self) {
        self.adc.cfgr.write(|w| {
            unsafe {
                w.res().bits(0b00);
            }
            w.cont().clear_bit();
            w.align().clear_bit();
            w.dmaen().clear_bit();
            w
        });
        self.adc.smpr2.write(|w| unsafe { w.smp15().bits(0b000) });
        self.adc.sqr4.write(|w| unsafe { w.sq15().bits(1) });
        self.adc.isr.write(|w| w.eos().set_bit());
        self.adc.cr.modify(|_, w| w.adstart().set_bit());
    }
}
