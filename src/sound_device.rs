use crate::data_reader::FileReader;
use crate::mp3_player::{Mp3Player, PlayError};
use cast::{u16, u32};
use log::{debug, error, info};
use rtfm::cyccnt::Instant;
use stm32f3xx_hal::stm32 as stm32f303;
use stm32f3xx_hal::stm32::Interrupt;
use stm32f3xx_hal::time;

pub(crate) const DMA_LENGTH: usize = 2 * (mp3::MAX_SAMPLES_PER_FRAME / 2);
const PLAYING_DONE_DELAY: u16 = 2_000; // ms

pub fn apb1enr() -> &'static stm32f303::rcc::APB1ENR {
    unsafe { &(*stm32f303::RCC::ptr()).apb1enr }
}

pub fn apb1rstr() -> &'static stm32f303::rcc::APB1RSTR {
    unsafe { &(*stm32f303::RCC::ptr()).apb1rstr }
}

fn ahbenr() -> &'static stm32f303::rcc::AHBENR {
    unsafe { &(*stm32f303::RCC::ptr()).ahbenr }
}

fn enable_dma2_ch3_interrupt() {
    unsafe { stm32f303::NVIC::unmask(Interrupt::DMA2_CH3) };
}

fn enable_tim7_interrupt() {
    unsafe { stm32f303::NVIC::unmask(Interrupt::TIM7) };
}

#[derive(PartialEq, Debug)]
pub enum DmaState {
    HalfTrigger,
    TriggerComplete,
    Stopped,
    Error,
    Unknown,
}

pub struct SoundDevice<'a> {
    dma_buffer: &'a mut [u16; DMA_LENGTH],
    stop_at_buffer_len: Option<u16>,
    dma2: stm32f303::DMA2,
    dac: stm32f303::DAC,
    tim2: stm32f303::TIM2,
    tim7: stm32f303::TIM7,
    sysclk_freq: time::Hertz,
    pub stopping: bool,
}

impl<'a> SoundDevice<'a> {
    pub fn new(
        dma_buffer: &'a mut [u16; DMA_LENGTH],
        sysclk: time::Hertz,
        tim2: stm32f303::TIM2,
        tim7: stm32f303::TIM7,
        dac: stm32f303::DAC,
        dma2: stm32f303::DMA2,
    ) -> Self {
        let apb1enr = apb1enr();
        let ahbenr = ahbenr();
        let obj = Self {
            dma_buffer,
            stop_at_buffer_len: None,
            tim2,
            tim7,
            dma2,
            dac,
            sysclk_freq: sysclk,
            stopping: false,
        };
        let apb1rstr = apb1rstr();
        obj.init_tim2(apb1enr, apb1rstr);
        obj.init_tim7(apb1enr, apb1rstr);
        obj.init_dac1(apb1enr);
        obj.init_dma2(ahbenr);
        obj
    }

    pub(crate) fn start_playing(
        &mut self,
        mp3_player: &mut Mp3Player,
        file_reader: &mut impl FileReader,
    ) -> Result<(), PlayError> {
        info!("start playing");
        self.fill_pcm_buffer(0, mp3_player, file_reader)?;
        self.fill_pcm_buffer(1, mp3_player, file_reader)?;
        let freq = mp3_player
            .last_frame_rate
            .ok_or(PlayError::NoValidMp3Frame)?;
        let arr = self.sysclk_freq.0 / freq.0;
        self.tim2.arr.write(|w| w.arr().bits(arr));
        self.tim2.cr1.modify(|_, w| w.cen().enabled());
        self.dma2.ch3.cr.modify(|_, w| w.en().enabled());
        self.stopping = false;
        Ok(())
    }

    pub fn fill_pcm_buffer(
        &mut self,
        buffer_index: usize,
        mp3_player: &mut Mp3Player,
        file_reader: &mut impl FileReader,
    ) -> Result<(), PlayError> {
        let buffer_len = self.dma_buffer.len();
        let dma_buffer_slice: &mut [u16] = match buffer_index {
            0 => &mut self.dma_buffer[0..buffer_len / 2],
            1 => &mut self.dma_buffer[buffer_len / 2..buffer_len],
            _ => panic!("Buffer index {} not expected", buffer_index),
        };
        let filled = mp3_player.next_frame(dma_buffer_slice);
        if filled < buffer_len / 2 {
            let filled = (filled + buffer_index * (buffer_len / 2)) as u16;
            info!(
                "Finishing music buffer_index: {}, filled:{}, now: {:?}",
                buffer_index,
                filled,
                Instant::now()
            );
            self.stop_at_buffer_len = Some(filled);
            if buffer_index == 1 || filled == 0 {
                self.set_dma_stop();
                debug!("Set dma stop");
            }
            Ok(())
        } else {
            mp3_player.fill_buffer(file_reader)
        }
    }

    pub fn dma_interrupt(&mut self) -> DmaState {
        // determine dma state
        // cache interrupt status before clearing interrupt flag
        let isr = self.dma2.isr.read();

        // clear interrupt flag and return dma state
        let mut state = if isr.teif3().is_error() {
            self.dma2.ifcr.write(|w| w.cteif3().clear());
            error!("Dma request error. Stop playing");
            self.stop_playing();
            DmaState::Error
        } else if isr.htif3().is_half() {
            self.dma2.ifcr.write(|w| w.chtif3().clear());
            DmaState::HalfTrigger
        } else if isr.tcif3().is_complete() {
            self.dma2.ifcr.write(|w| w.ctcif3().clear());
            DmaState::TriggerComplete
        } else {
            DmaState::Unknown
        };

        if !self.stopping
            && self.stop_at_buffer_len.is_some()
            && (state == DmaState::TriggerComplete || state == DmaState::HalfTrigger)
        {
            self.set_dma_stop();
            state = DmaState::Stopped;
        }
        state
    }

    pub fn playing_stop_timer_interrupt(&self) {
        self.tim7.sr.write(|w| w.uif().clear_bit());
        self.stop_playing();
    }

    pub fn stop_playing(&self) {
        debug!("Stop playing");
        self.tim2.cr1.modify(|_, w| w.cen().disabled());
        self.dma2.ch3.cr.modify(|_, w| w.en().disabled());
    }

    pub fn set_dma_stop(&mut self) {
        let stop_index = self.stop_at_buffer_len.expect("To have stop_index set");
        self.stopping = true;
        self.dma2.ch3.cr.modify(|_, w| w.en().disabled());
        if stop_index != 0 {
            self.dma2.ch3.ndtr.write(|w| w.ndt().bits(stop_index));
        }
        self.dma2.ch3.cr.modify(|_, w| {
            w.circ().disabled() // dma mode is circular
        });
        self.dma2.ch3.cr.modify(|_, w| w.en().enabled());
        self.trigger_playing_stop_timer();
    }

    fn init_tim2(&self, apb1enr: &stm32f303::rcc::APB1ENR, apb1rstr: &stm32f303::rcc::APB1RSTR) {
        apb1rstr.modify(|_, w| w.tim2rst().reset());
        apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
        apb1enr.modify(|_, w| w.tim2en().set_bit());
        self.tim2.cr2.write(|w| w.mms().update());
    }

    fn init_tim7(&self, apb1enr: &stm32f303::rcc::APB1ENR, apb1rstr: &stm32f303::rcc::APB1RSTR) {
        apb1rstr.modify(|_, w| w.tim7rst().reset());
        apb1rstr.modify(|_, w| w.tim7rst().clear_bit());
        apb1enr.modify(|_, w| w.tim7en().set_bit());
        self.tim7.cr1.write(|w| w.opm().enabled().cen().disabled());
        let psc_ms = u16((self.sysclk_freq.0 / 72000) - 1).unwrap();
        let arr = PLAYING_DONE_DELAY;
        self.tim7.psc.write(|w| w.psc().bits(psc_ms));
        self.tim7.arr.write(|w| unsafe { w.bits(u32(arr)) });
        self.tim7.egr.write(|w| w.ug().update());
        self.tim7.sr.modify(|_, w| w.uif().clear());
        self.tim7.dier.write(|w| w.uie().enabled());
        enable_tim7_interrupt();
    }

    fn trigger_playing_stop_timer(&self) {
        self.tim7.cr1.modify(|_, w| w.cen().enabled());
    }

    fn init_dac1(&self, apb1enr: &stm32f303::rcc::APB1ENR) {
        apb1enr.modify(|_, w| w.dac1en().set_bit());
        self.dac.cr.write(|w| {
            w.boff1().disabled();
            w.ten1().enabled();
            w.tsel1().tim2_trgo();
            w
        });
        self.dac.cr.modify(|_, w| w.en1().enabled());
    }

    pub fn init_dma2(&self, ahbenr: &stm32f303::rcc::AHBENR) {
        ahbenr.modify(|_, w| w.dma2en().set_bit());
        let ma = self.dma_buffer.as_ptr() as usize as u32;
        let pa = &self.dac.dhr12r1 as *const stm32f303::dac::DHR12R1 as usize as u32;
        let ndt = self.dma_buffer.len() as u16;

        self.dma2.ch3.mar.write(|w| w.ma().bits(ma));
        self.dma2.ch3.par.write(|w| w.pa().bits(pa));
        self.dma2.ch3.ndtr.write(|w| w.ndt().bits(ndt));
        self.dma2.ch3.cr.write(|w| {
            w.dir().from_memory(); // source is memory
            w.mem2mem().disabled(); // disable memory to memory transfer
            w.minc().enabled(); // increment memory address every transfer
            w.pinc().disabled(); // don't increment peripheral address every transfer
            w.msize().bits16(); // memory word size is 32 bits
            w.psize().bits16(); // peripheral word size is 32 bits
            w.circ().enabled(); // dma mode is circular
            w.pl().high(); // set dma priority to high
            w.teie().enabled(); // trigger an interrupt if an error occurs
            w.tcie().enabled(); // trigger an interrupt when transfer is complete
            w.htie().enabled(); // trigger an interrupt when half the transfer is complete
            w
        });
        enable_dma2_ch3_interrupt();
        self.dac.cr.modify(|_, w| w.dmaen1().enabled());
    }
}
