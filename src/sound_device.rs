use crate::data_reader::{DataReader, DirectoryNavigator};
use crate::mp3_player::{Mp3Player, PlayError};
use log::{error, info};
use rtfm::cyccnt::Instant;
use stm32f3xx_hal::stm32 as stm32f303;
use stm32f3xx_hal::stm32::Interrupt;
use stm32f3xx_hal::time;

pub(crate) const DMA_LENGTH: usize = 2 * (mp3::MAX_SAMPLES_PER_FRAME / 2);

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

pub struct SoundDevice<'a> {
    dma_buffer: &'a mut [u16; DMA_LENGTH],
    stop_at_buffer_len: Option<usize>,
    dma2: stm32f303::DMA2,
    dac: stm32f303::DAC,
    tim2: stm32f303::TIM2,
    sysclk_freq: time::Hertz,
    pub stopping: bool,
    pub debug_data: DebuggingData,
}

impl<'a> SoundDevice<'a> {
    pub fn new(
        dma_buffer: &'a mut [u16; DMA_LENGTH],
        sysclk: time::Hertz,
        tim2: stm32f303::TIM2,
        dac: stm32f303::DAC,
        dma2: stm32f303::DMA2,
    ) -> Self {
        let apb1enr = apb1enr();
        let ahbenr = ahbenr();
        let obj = Self {
            dma_buffer,
            stop_at_buffer_len: None,
            tim2,
            dma2,
            dac,
            sysclk_freq: sysclk,
            stopping: false,
            debug_data: DebuggingData::new(),
        };
        let apb1rstr = apb1rstr();
        obj.init_tim2(apb1enr, apb1rstr);
        obj.init_dac1(apb1enr);
        obj.init_dma2(ahbenr);
        obj
    }

    pub(crate) fn start_playing(
        &mut self,
        mp3_player: &mut Mp3Player,
        data_reader: &mut DataReader,
        directory_navigator: &mut impl DirectoryNavigator,
    ) -> Result<(), PlayError> {
        info!("start playing");
        self.stop_playing();
        self.fill_pcm_buffer(0, mp3_player, data_reader, directory_navigator)?;
        self.fill_pcm_buffer(1, mp3_player, data_reader, directory_navigator)?;
        let freq = mp3_player
            .last_frame_rate
            .ok_or(PlayError::NoValidMp3Frame)?;
        let arr = self.sysclk_freq.0 / freq.0;
        self.tim2.arr.write(|w| w.arr().bits(arr));
        self.tim2.cr1.modify(|_, w| w.cen().enabled());
        self.dma2.ch3.cr.modify(|_, w| {
            w.circ().enabled();
            w.tcie().enabled();
            w.htie().enabled();
            w.en().enabled()
        });
        let ndt = self.dma_buffer.len() as u16;
        self.dma2.ch3.ndtr.write(|w| w.ndt().bits(ndt));
        self.stopping = false;
        self.stop_at_buffer_len = None;
        Ok(())
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
                "Finishing music buffer_index: {}, filled:{}, now: {:?}",
                buffer_index,
                filled,
                Instant::now()
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
    pub fn toggle_pause(&self) {
        self.tim2.cr1.modify(|r, w| w.cen().bit(!r.cen().bit()));
    }

    pub fn stop_playing(&self) {
        self.tim2.cr1.modify(|_, w| w.cen().disabled());
        self.dma2.ch3.cr.modify(|_, w| w.en().disabled());
    }

    pub fn set_dma_stop(&mut self, state: DmaState) -> DmaState {
        self.stopping = true;
        let stop_index = self.stop_at_buffer_len.expect("To have stop_index set");
        self.dma2.ch3.cr.modify(|_, w| w.en().disabled());
        if (stop_index == 0 && state == DmaState::TriggerComplete)
            || (stop_index == self.dma_buffer.len() / 2 && state == DmaState::HalfTrigger)
        {
            self.stop_playing();
            DmaState::Stopped
        } else {
            self.dma2.ch3.cr.modify(|_, w| {
                w.tcie().enabled(); // trigger an interrupt when transfer is complete
                w.htie().disabled(); // trigger an interrupt when half the transfer is complete
                w.circ().disabled() // dma mode is circular
            });
            self.dma2
                .ch3
                .ndtr
                .write(|w| w.ndt().bits(stop_index as u16));
            self.dma2.ch3.cr.modify(|_, w| w.en().enabled());
            state
        }
    }

    fn init_tim2(&self, apb1enr: &stm32f303::rcc::APB1ENR, apb1rstr: &stm32f303::rcc::APB1RSTR) {
        apb1rstr.modify(|_, w| w.tim2rst().reset());
        apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
        apb1enr.modify(|_, w| w.tim2en().set_bit());
        self.tim2.cr2.write(|w| w.mms().update());
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
            w.pl().high(); // set dma priority to high
            w.teie().enabled(); // trigger an interrupt if an error occurs
            w.circ().enabled(); // dma mode is circular
            w.tcie().enabled(); // trigger an interrupt when transfer is complete
            w.htie().enabled(); // trigger an interrupt when half the transfer is complete
            w
        });
        enable_dma2_ch3_interrupt();
        self.dac.cr.modify(|_, w| w.dmaen1().enabled());
    }
}
