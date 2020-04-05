use crate::data_reader::FileReader;
use crate::mp3_player::{Mp3Player, PlayError};
use log::info;
use stm32f3xx_hal::stm32 as stm32f303;
use stm32f3xx_hal::stm32::Interrupt;
use stm32f3xx_hal::time;

pub(crate) const DMA_LENGTH: usize = 2 * (mp3::MAX_SAMPLES_PER_FRAME / 2);
fn apb1enr() -> &'static stm32f303::rcc::APB1ENR {
    unsafe { &(*stm32f303::RCC::ptr()).apb1enr }
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
    Error,
    Unknown,
}

pub struct SoundDevice<'a> {
    dma_buffer: &'a mut [u16; DMA_LENGTH],
    stop_at_buffer_len: Option<u16>,
    dma2: stm32f303::DMA2,
    dac: stm32f303::DAC,
    tim2: stm32f303::TIM2,
    sysclk_freq: time::Hertz,
    pub playing: bool,
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
            playing: false,
        };
        obj.init_tim2(apb1enr);
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
        self.playing = true;
        Ok(())
    }

    pub fn stop_playing(&mut self) {
        self.tim2.cr1.modify(|_, w| w.cen().disabled());
        self.dma2.ch3.cr.modify(|_, w| w.en().disabled());
        self.playing = false
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
            info!("Finishing music {}, {}", buffer_index, filled);
            self.stop_at_buffer_len = Some(filled as u16);
            if buffer_index == 1 {
                self.set_dma_stop();
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
        let state = if isr.teif3().is_error() {
            self.dma2.ifcr.write(|w| w.cteif3().clear());
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

        if state == DmaState::TriggerComplete && self.playing {
            self.set_dma_stop();
        }

        state
    }

    fn set_dma_stop(&mut self) {
        if let Some(stop_index) = self.stop_at_buffer_len {
            self.dma2.ch3.cr.modify(|_, w| w.en().disabled());
            self.dma2.ch3.ndtr.write(|w| w.ndt().bits(stop_index));
            self.dma2.ch3.cr.write(|w| {
                w.circ().disabled() // dma mode is circular
            });
            self.dma2.ch3.cr.modify(|_, w| w.en().enabled());
            self.playing = false;
        }
    }

    fn init_tim2(&self, apb1enr: &stm32f303::rcc::APB1ENR) {
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
