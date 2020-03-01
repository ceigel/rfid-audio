#![no_std]
#![no_main]

extern crate embedded_mp3 as mp3;
use panic_itm as _;

use cortex_m::peripheral::ITM;
use cortex_m::{iprint, iprintln};
use rtfm::app;
use rtfm::cyccnt::{Instant, U32Ext as _};
use stm32f3xx_hal::gpio::{gpioa, Analog};
use stm32f3xx_hal::stm32 as stm32f303;
use stm32f3xx_hal::stm32::Interrupt;
use stm32f3xx_hal::{prelude::*, time};

const DMA_LENGTH: usize = 2 * (mp3::MAX_SAMPLES_PER_FRAME / 2);
struct Buffers {
    pub dma_buffer: [u16; DMA_LENGTH],
    pub mp3_decoder_data: mp3::DecoderData,
}

impl Buffers {
    pub const fn new() -> Self {
        Self {
            dma_buffer: [0; DMA_LENGTH],
            mp3_decoder_data: mp3::DecoderData::new(),
        }
    }
}

static mut BUFFERS: Buffers = Buffers::new();

fn apb1enr() -> &'static stm32f303::rcc::APB1ENR {
    unsafe { &(*stm32f303::RCC::ptr()).apb1enr }
}

fn ahbenr() -> &'static stm32f303::rcc::AHBENR {
    unsafe { &(*stm32f303::RCC::ptr()).ahbenr }
}

fn enable_dma2_ch3_interrupt() {
    unsafe { stm32f303::NVIC::unmask(Interrupt::DMA2_CH3) };
}

pub struct Mp3DataProvider {
    mp3_data: &'static [u8],
    read_index: usize,
}

impl Mp3DataProvider {
    pub fn new(mp3_data: &'static [u8]) -> Self {
        Self {
            mp3_data,
            read_index: 0,
        }
    }

    pub fn get_buffer(&self) -> &'static [u8] {
        &self.mp3_data[self.read_index..]
    }

    pub fn advance_index(&mut self, offset: usize) {
        self.read_index += offset;
    }
}

pub struct Mp3Decoder<'a> {
    decoder: mp3::Decoder<'a>,
    last_frame_rate: Option<time::Hertz>,
    data_provider: Mp3DataProvider,
}

impl<'a> Mp3Decoder<'a> {
    pub fn new(mp3_data: &'a mut mp3::DecoderData, data_provider: Mp3DataProvider) -> Self {
        Self {
            decoder: mp3::Decoder::new(mp3_data),
            last_frame_rate: None,
            data_provider,
        }
    }

    pub fn next_frame(&mut self, pcm_buffer: &mut [u16]) -> usize {
        let mp3: &[u8] = self.data_provider.get_buffer();
        match self.decoder.decode(mp3) {
            mp3::DecodeResult::Successful(bytes_read, frame) => {
                self.data_provider.advance_index(bytes_read);
                self.last_frame_rate.replace(time::Hertz(frame.sample_rate));
                let samples = frame
                    .samples
                    .iter()
                    .step_by(frame.channels as usize)
                    .take(frame.sample_count as usize)
                    .map(|sample| ((*sample as i32) - (core::i16::MIN as i32)) as u16 >> 4);
                let mut index: usize = 0;
                for val in samples {
                    pcm_buffer[index] = val;
                    index += 1;
                }
                index
            }
            mp3::DecodeResult::SkippedData(skipped_data) => {
                self.data_provider.advance_index(skipped_data);
                self.next_frame(pcm_buffer)
            }
            mp3::DecodeResult::InsufficientData => 0,
        }
    }
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
    mp3_decoder: Mp3Decoder<'a>,
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
        mp3_decoder: Mp3Decoder<'a>,
        sysclk: time::Hertz,
        tim2: stm32f303::TIM2,
        dac: stm32f303::DAC,
        dma2: stm32f303::DMA2,
    ) -> Self {
        let apb1enr = apb1enr();
        let ahbenr = ahbenr();
        let obj = Self {
            dma_buffer,
            mp3_decoder,
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

    pub fn start_playing(&mut self) {
        if let Some(freq) = self.mp3_decoder.last_frame_rate {
            let arr = self.sysclk_freq.0 / freq.0;
            self.tim2.arr.write(|w| w.arr().bits(arr));
            self.tim2.cr1.modify(|_, w| w.cen().enabled());
            self.dma2.ch3.cr.modify(|_, w| w.en().enabled());
            self.playing = true
        }
    }

    pub fn stop_playing(&mut self) {
        self.tim2.cr1.modify(|_, w| w.cen().disabled());
        self.dma2.ch3.cr.modify(|_, w| w.en().disabled());
        self.playing = false
    }

    pub fn fill_pcm_buffer(&mut self, buffer_index: usize, itm: &mut ITM) {
        let buffer_len = self.dma_buffer.len();
        let dma_buffer_slice: &mut [u16] = match buffer_index {
            0 => &mut self.dma_buffer[0..buffer_len / 2],
            1 => &mut self.dma_buffer[buffer_len / 2..buffer_len],
            _ => panic!("Buffer index {} not expected", buffer_index),
        };
        let filled = self.mp3_decoder.next_frame(dma_buffer_slice);
        if filled < buffer_len / 2 {
            iprintln!(
                &mut itm.stim[0],
                "Finishing music {}, {}",
                buffer_index,
                filled
            );
            self.stop_at_buffer_len = Some(filled as u16);
            if buffer_index == 1 {
                self.set_dma_stop();
            }
        }
        if filled == 0 {
            self.stop_playing();
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

fn init_clocks(
    cfgr: stm32f3xx_hal::rcc::CFGR,
    mut flash: stm32f3xx_hal::flash::Parts,
) -> stm32f3xx_hal::rcc::Clocks {
    cfgr.use_hse(8.mhz())
        .sysclk(72.mhz())
        .hclk(72.mhz())
        .pclk2(72.mhz())
        .pclk1(36.mhz())
        .freeze(&mut flash.acr)
}

#[app(device = stm32f3xx_hal::stm32, monotonic = rtfm::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        itm: ITM,
        sound_device: SoundDevice<'static>,
        pa4: gpioa::PA4<Analog>,
    }
    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let mut core = cx.core;
        let mut itm = core.ITM;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        let device = cx.device;

        let flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let clocks = init_clocks(rcc.cfgr, flash);

        let mut gpioa = device.GPIOA.split(&mut rcc.ahb);
        let pa4 = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

        let mp3_file = include_bytes!("intro.mp3");
        let data_provider = Mp3DataProvider::new(&mp3_file[..]);
        let buffers = unsafe { &mut BUFFERS };
        let mp3_decoder: Mp3Decoder = Mp3Decoder::new(&mut buffers.mp3_decoder_data, data_provider);

        let mut sound_device = SoundDevice::new(
            &mut buffers.dma_buffer,
            mp3_decoder,
            clocks.sysclk(),
            device.TIM2,
            device.DAC,
            device.DMA2,
        );

        iprintln!(&mut itm.stim[0], "Init finished");
        sound_device.fill_pcm_buffer(0, &mut itm);
        sound_device.fill_pcm_buffer(1, &mut itm);
        sound_device.start_playing();
        iprintln!(&mut itm.stim[0], "Started playing");

        init::LateResources {
            itm,
            pa4,
            sound_device,
        }
    }

    #[idle(resources=[itm, sound_device])]
    fn idle(mut cx: idle::Context) -> ! {
        static mut TICK: Option<Instant> = None;
        static mut ALREADY_STOPPED: bool = false;
        TICK.replace(Instant::now());
        cx.resources
            .itm
            .lock(|itm| iprintln!(&mut itm.stim[0], "idle"));
        loop {
            if TICK.unwrap().elapsed().as_cycles() > 4_000_000 {
                *TICK = Some(Instant::now());
                let playing = cx.resources.sound_device.lock(|sd| sd.playing);
                cx.resources.itm.lock(|itm| {
                    if !playing && !*ALREADY_STOPPED {
                        *ALREADY_STOPPED = true;
                        iprintln!(&mut itm.stim[0], "Music stopped");
                    }
                });
            }
        }
    }

    #[task(capacity=1, priority=8, resources=[itm, sound_device])]
    fn process_dma_request(cx: process_dma_request::Context, new_state: DmaState) {
        match new_state {
            DmaState::Unknown => panic!("Unknown dma state"),
            DmaState::Error => {
                iprintln!(
                    &mut cx.resources.itm.stim[0],
                    "Dma request error. Stop playing"
                );
                cx.resources.sound_device.stop_playing();
            }
            DmaState::HalfTrigger | DmaState::TriggerComplete => {
                let index = if new_state == DmaState::HalfTrigger {
                    0
                } else {
                    1
                };
                cx.resources
                    .sound_device
                    .fill_pcm_buffer(index, cx.resources.itm);
            }
        }
    }

    #[task(binds = DMA2_CH3, priority=8, resources=[sound_device, itm], spawn=[process_dma_request])]
    fn dma2_ch3(cx: dma2_ch3::Context) {
        let state = cx.resources.sound_device.dma_interrupt();
        if cx.spawn.process_dma_request(state).is_err() {
            cx.resources.sound_device.stop_playing();
            iprintln!(&mut cx.resources.itm.stim[0], "Spawn error. Stop playing!");
        }
    }
    extern "C" {
        fn SPI1();
    }
};
