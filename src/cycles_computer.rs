use crate::hal::time::Hertz;
use core::time;
use rtic::cyccnt;

pub struct CyclesComputer {
    frequency: Hertz,
}

impl CyclesComputer {
    pub fn new(frequency: Hertz) -> Self {
        CyclesComputer { frequency }
    }

    pub fn to_cycles(&self, duration: time::Duration) -> cyccnt::Duration {
        use rtic::cyccnt::U32Ext;
        let s_part = (duration.as_secs() as u32) * self.frequency.0;
        let mms_part = (duration.subsec_millis()) * (self.frequency.0 / 1000);
        (s_part + mms_part).cycles()
    }

    pub fn from_cycles(&self, ticks: cyccnt::Duration) -> time::Duration {
        let Hertz(hz) = self.frequency;
        let time = (ticks.as_cycles() as f32) / (hz as f32);
        time::Duration::from_secs_f32(time)
    }
}
