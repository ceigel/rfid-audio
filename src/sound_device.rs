use crate::data_reader::{DataReader, DirectoryNavigator};
use crate::mp3_player::{Mp3Player, PlayError};
use log::{error, info};
use rtic::cyccnt::{Duration, Instant};
use stm32l4xx_hal::gpio::{gpioa, Output, PushPull};
use stm32l4xx_hal::prelude::OutputPin;
use stm32l4xx_hal::stm32 as stm32l431;
use stm32l4xx_hal::stm32::Interrupt;
use stm32l4xx_hal::time;

pub(crate) const DMA_LENGTH: usize = 2 * (mp3::MAX_SAMPLES_PER_FRAME / 2);

pub fn apb1enr() -> &'static stm32l431::rcc::APB1ENR1 {
    unsafe { &(*stm32l431::RCC::ptr()).apb1enr1 }
}

pub fn apb1rstr() -> &'static stm32l431::rcc::APB1RSTR1 {
    unsafe { &(*stm32l431::RCC::ptr()).apb1rstr1 }
}

fn ahbenr() -> &'static stm32l431::rcc::AHB1ENR {
    unsafe { &(*stm32l431::RCC::ptr()).ahb1enr }
}

fn enable_dma1_ch3_interrupt() {
    unsafe { stm32l431::NVIC::unmask(Interrupt::DMA1_CH3) };
}

#[derive(PartialEq, Debug)]
pub enum DmaState {
    HalfTrigger,
    TriggerComplete,
    Stopped,
    Error,
    Unknown,
}

#[derive(Debug)]
pub struct DebuggingData {
    pub trigger_complete_count: u32,
    pub half_trigger_count: u32,
    pub unknown_count: u32,
}

impl DebuggingData {
    pub fn new() -> Self {
        DebuggingData {
            trigger_complete_count: 0,
            half_trigger_count: 0,
            unknown_count: 0,
        }
    }
}

pub struct InnerState {
    audio_en: gpioa::PA12<Output<PushPull>>,
    stopped: bool,
}

impl InnerState {
    pub fn new(mut audio_en: gpioa::PA12<Output<PushPull>>) -> Self {
        audio_en.set_low().expect("To be able to re-set audio_en");

        Self {
            audio_en,
            stopped: true,
        }
    }
    pub fn set_playing(&mut self) {
        self.stopped = false;
        self.audio_en
            .set_high()
            .expect("To be able to set audio_en");
    }
    pub fn set_stopped(&mut self) {
        self.stopped = true;
        self.audio_en.set_low().expect("To be able to set audio_en");
    }
    pub fn is_playing(&self) -> bool {
        !self.stopped
    }
}

pub struct SoundDevice<'a> {
    dma_buffer: &'a mut [u16; DMA_LENGTH],
    stop_at_buffer_len: Option<usize>,
    dma1: stm32l431::DMA1,
    dac: stm32l431::DAC1,
    tim6: stm32l431::TIM6,
    sysclk_freq: time::Hertz,
    pub stopping: bool,
    pub debug_data: DebuggingData,
    play_pause_start: Option<Instant>,
    inner_state: InnerState,
}

impl<'a> SoundDevice<'a> {
    pub fn new(
        dma_buffer: &'a mut [u16; DMA_LENGTH],
        sysclk: time::Hertz,
        tim6: stm32l431::TIM6,
        dac: stm32l431::DAC1,
        dma1: stm32l431::DMA1,
        audio_en: gpioa::PA12<Output<PushPull>>,
    ) -> Self {
        let apb1enr = apb1enr();
        let ahbenr = ahbenr();
        let inner_state = InnerState::new(audio_en);
        let obj = Self {
            dma_buffer,
            stop_at_buffer_len: None,
            tim6,
            dma1,
            dac,
            sysclk_freq: sysclk,
            stopping: false,
            debug_data: DebuggingData::new(),
            play_pause_start: None,
            inner_state,
        };
        let apb1rstr = apb1rstr();
        obj.init_tim6(apb1enr, apb1rstr);
        obj.init_dac1(apb1enr);
        obj.init_dma1(ahbenr);
        obj
    }

    pub(crate) fn start_playing(
        &mut self,
        mp3_player: &mut Mp3Player,
        data_reader: &mut DataReader,
        directory_navigator: &mut impl DirectoryNavigator,
    ) -> Result<(), PlayError> {
        info!("start playing called");
        self.stop_playing();
        self.fill_pcm_buffer(0, mp3_player, data_reader, directory_navigator)?;
        self.fill_pcm_buffer(1, mp3_player, data_reader, directory_navigator)?;
        let freq = mp3_player
            .last_frame_rate
            .ok_or(PlayError::NoValidMp3Frame)?;
        let arr = (self.sysclk_freq.0 / freq.0) as u16;
        self.tim6.arr.write(|w| w.arr().bits(arr));
        self.tim6.cr1.modify(|_, w| w.cen().enabled());
        self.dma1.ccr3.modify(|_, w| {
            w.circ().enabled();
            w.tcie().enabled();
            w.htie().enabled();
            w.en().enabled()
        });
        let ndt = self.dma_buffer.len() as u16;
        self.dma1.cndtr3.write(|w| w.ndt().bits(ndt));
        self.stopping = false;
        self.stop_at_buffer_len = None;
        self.play_pause_start.replace(Instant::now());
        self.inner_state.set_playing();
        Ok(())
    }

    pub fn is_playing(&self) -> bool {
        self.inner_state.is_playing()
    }
    pub fn play_pause_elapsed(&self) -> Option<Duration> {
        self.play_pause_start.and_then(|t| {
            if Instant::now() < t {
                None
            } else {
                Some(t.elapsed())
            }
        })
    }

    pub fn fill_pcm_buffer(
        &mut self,
        buffer_index: usize,
        mp3_player: &mut Mp3Player,
        data_reader: &mut DataReader,
        directory_navigator: &mut impl DirectoryNavigator,
    ) -> Result<(), PlayError> {
        let buffer_len = self.dma_buffer.len();
        let dma_buffer_slice: &mut [u16] = match buffer_index {
            0 => &mut self.dma_buffer[0..buffer_len / 2],
            1 => &mut self.dma_buffer[buffer_len / 2..buffer_len],
            _ => panic!("Buffer index {} not expected", buffer_index),
        };
        let filled = mp3_player.next_frame(dma_buffer_slice);
        let end_index = filled + buffer_index * (buffer_len / 2);
        if filled < buffer_len / 2 {
            info!(
                "Finishing music buffer_index: {}, filled:{}",
                buffer_index, filled,
            );
            self.stop_at_buffer_len = Some(end_index);
            Ok(())
        } else {
            mp3_player
                .fill_buffer(data_reader, directory_navigator)
                .map_err(|e| {
                    self.stop_playing();
                    e
                })
        }
    }

    pub fn dma_interrupt(&mut self) -> DmaState {
        // determine dma state
        // cache interrupt status before clearing interrupt flag
        let isr = self.dma1.isr.read();

        // clear interrupt flag and return dma state
        let mut state = if isr.teif3().is_error() {
            self.dma1.ifcr.write(|w| w.cteif3().clear());
            error!("Dma request error. Stop playing");
            self.stop_playing();
            DmaState::Error
        } else if isr.htif3().is_half() {
            self.dma1.ifcr.write(|w| w.chtif3().clear());
            DmaState::HalfTrigger
        } else if isr.tcif3().is_complete() {
            self.dma1.ifcr.write(|w| w.ctcif3().clear());
            DmaState::TriggerComplete
        } else {
            DmaState::Unknown
        };

        if self.stopping {
            self.stop_playing();
            state = DmaState::Stopped;
        } else if self.stop_at_buffer_len.is_some()
            && (state == DmaState::TriggerComplete || state == DmaState::HalfTrigger)
        {
            state = self.set_dma_stop(state);
        }
        state
    }
    pub fn toggle_pause(&mut self) {
        self.tim6.cr1.modify(|r, w| w.cen().bit(!r.cen().bit()));
        if self.tim6.cr1.read().cen().is_enabled() {
            self.inner_state.set_playing();
        } else {
            self.inner_state.set_stopped();
        }
    }

    pub fn stop_playing(&mut self) {
        self.tim6.cr1.modify(|_, w| w.cen().disabled());
        self.dma1.ccr3.modify(|_, w| w.en().disabled());
        self.play_pause_start.take();
        self.inner_state.set_stopped();
    }

    pub fn set_dma_stop(&mut self, state: DmaState) -> DmaState {
        self.stopping = true;
        let stop_index = self.stop_at_buffer_len.expect("To have stop_index set");
        self.dma1.ccr3.modify(|_, w| w.en().disabled());
        if (stop_index == 0 && state == DmaState::TriggerComplete)
            || (stop_index == self.dma_buffer.len() / 2 && state == DmaState::HalfTrigger)
        {
            self.stop_playing();
            DmaState::Stopped
        } else {
            self.dma1.ccr3.modify(|_, w| {
                w.tcie().enabled(); // trigger an interrupt when transfer is complete
                w.htie().disabled(); // trigger an interrupt when half the transfer is complete
                w.circ().disabled() // dma mode is circular
            });
            self.dma1.cndtr3.write(|w| w.ndt().bits(stop_index as u16));
            self.dma1.ccr3.modify(|_, w| w.en().enabled());
            state
        }
    }

    fn init_tim6(&self, apb1enr: &stm32l431::rcc::APB1ENR1, apb1rstr: &stm32l431::rcc::APB1RSTR1) {
        apb1rstr.modify(|_, w| w.tim6rst().set_bit());
        apb1rstr.modify(|_, w| w.tim6rst().clear_bit());
        apb1enr.modify(|_, w| w.tim6en().set_bit());
        self.tim6.cr2.write(|w| w.mms().update());
    }

    fn init_dac1(&self, apb1enr: &stm32l431::rcc::APB1ENR1) {
        apb1enr.modify(|_, w| w.dac1en().set_bit());
        self.dac.mcr.write(|w| {
            unsafe {
                w.mode1().bits(0b000); // normal mode, buffer enabled
            }
            w
        });
        self.dac.cr.write(|w| {
            w.ten1().set_bit();
            unsafe {
                w.tsel1().bits(0b000); // TIM6_TRGO
            }
            w
        });
        self.dac.cr.modify(|_, w| w.en1().set_bit());
    }

    pub fn init_dma1(&self, ahbenr: &stm32l431::rcc::AHB1ENR) {
        ahbenr.modify(|_, w| w.dma1en().set_bit());
        let ma = self.dma_buffer.as_ptr() as usize as u32;
        let pa = &self.dac.dhr12r1 as *const stm32l431::dac1::DHR12R1 as usize as u32;
        let ndt = self.dma_buffer.len() as u16;

        self.dma1.cmar3.write(|w| unsafe { w.ma().bits(ma) });
        self.dma1.cpar3.write(|w| unsafe { w.pa().bits(pa) });
        self.dma1.cndtr3.write(|w| w.ndt().bits(ndt));
        self.dma1.ccr3.write(|w| {
            w.dir().from_memory(); // source is memory
            w.mem2mem().disabled(); // disable memory to memory transfer
            w.minc().enabled(); // increment memory address every transfer
            w.pinc().disabled(); // don't increment peripheral address every transfer
            w.msize().bits16(); // memory word size is 32 bits
            w.psize().bits16(); // peripheral word size is 32 bits
            w.pl().high(); // set dma priority to high
            w.teie().enabled(); // trigger an interrupt if an error occurs
            w.circ().enabled(); // dma mode is circular
            w.tcie().enabled(); // trigger an interrupt when transfer is complete
            w.htie().enabled(); // trigger an interrupt when half the transfer is complete
            w
        });
        enable_dma1_ch3_interrupt();
        self.dma1.cselr.write(|w| w.c3s().bits(0b0110));
        self.dac.cr.modify(|_, w| w.dmaen1().set_bit());
    }
}
