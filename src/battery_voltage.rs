#![allow(dead_code)]
use crate::hal::adc::ADC;
use crate::hal::gpio::{gpiob, Analog};
use crate::hal::prelude::*;

const REGULATED_VOLTAGE: u32 = 3300;
const VOLTAGE_DIVIDER_VAL: u32 = 2;
const ADC_OFFSET: u32 = 65;

#[allow(dead_code)]
pub struct BatteryReader {
    pin: gpiob::PB0<Analog>,
    adc: ADC,
}

impl BatteryReader {
    pub fn new(adc: ADC, pb0: gpiob::PB0<Analog>) -> Self {
        BatteryReader { pin: pb0, adc }
    }

    pub fn read(&mut self) -> u16 {
        let adc_read: u32 = self
            .adc
            .read(&mut self.pin)
            .map(|v| v as u32 + ADC_OFFSET)
            .unwrap_or(u32::MAX);
        ((adc_read * REGULATED_VOLTAGE * VOLTAGE_DIVIDER_VAL) / (self.adc.get_max_value() as u32))
            as u16
    }
}
