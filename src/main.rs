#![no_std]
#![no_main]

extern crate embedded_mp3 as mp3;
use panic_itm as _;

use cortex_m_log::destination;
use cortex_m_log::log::Logger;
use cortex_m_log::printer::itm::InterruptSync;
use embedded_sdmmc as sdmmc;
use hal::gpio::{gpioa, Analog, Floating, Input, Output, PushPull, AF5};
use hal::hal as embedded_hal;
use hal::spi::{Phase, Polarity, Spi};
use log::{error, info};
use rtfm::app;
use rtfm::cyccnt::Instant;
use stm32f3xx_hal as hal;
use stm32f3xx_hal::stm32 as stm32f303;
use stm32f3xx_hal::stm32::Interrupt;
use stm32f3xx_hal::{prelude::*, time};

const DMA_LENGTH: usize = 2 * (mp3::MAX_SAMPLES_PER_FRAME / 2);
const MP3_DATA_LENGTH: usize = 16384;
const MP3_MAX_READ: usize = MP3_DATA_LENGTH / 8;
static mut LOGGER: Option<Logger<InterruptSync>> = None;
const INTRO_FILE_NAME: &str = "intro.mp3";

struct CCRamBuffers {
    pub mp3_decoder_data: mp3::DecoderData,
}

struct Buffers {
    pub dma_buffer: [u16; DMA_LENGTH],
    pub pcm_buffer: [i16; mp3::MAX_SAMPLES_PER_FRAME],
    pub mp3_data: [u8; MP3_DATA_LENGTH],
}

impl CCRamBuffers {
    pub const fn new() -> Self {
        Self {
            mp3_decoder_data: mp3::DecoderData::new(),
        }
    }
}

impl Buffers {
    pub const fn new() -> Self {
        Self {
            dma_buffer: [0; DMA_LENGTH],
            pcm_buffer: [0; mp3::MAX_SAMPLES_PER_FRAME],
            mp3_data: [0; MP3_DATA_LENGTH],
        }
    }
}

static mut BUFFERS: Buffers = Buffers::new();
#[link_section = ".ccram_data"]
static mut CCRAM_BUFFERS: CCRamBuffers = CCRamBuffers::new();

struct DummyTimeSource {}
impl sdmmc::TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> sdmmc::Timestamp {
        sdmmc::Timestamp::from_calendar(2020, 3, 7, 13, 23, 0).expect("To create date")
    }
}

type FileError = sdmmc::Error<sdmmc::SdMmcError>;
pub struct DataReader {
    file: sdmmc::File,
    file_name: sdmmc::ShortFileName,
}

impl DataReader {
    pub fn new(file: sdmmc::File, file_name: sdmmc::ShortFileName) -> Self {
        DataReader { file, file_name }
    }

    pub fn read_data(
        &mut self,
        file_reader: &mut impl FileReader,
        out: &mut [u8],
    ) -> Result<usize, FileError> {
        file_reader.read_data(&mut self.file, out)
    }

    pub fn file_name(&self) -> sdmmc::ShortFileName {
        self.file_name.clone()
    }
}

pub trait FileReader {
    fn read_data(&mut self, file: &mut sdmmc::File, out: &mut [u8]) -> Result<usize, FileError>;
}

pub struct SdCardReader<SPI, CS>
where
    SPI: embedded_hal::spi::FullDuplex<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::spi::FullDuplex<u8>>::Error: core::fmt::Debug,
{
    controller: sdmmc::Controller<sdmmc::SdMmcSpi<SPI, CS>, DummyTimeSource>,
    volume: sdmmc::Volume,
    root_dir: sdmmc::Directory,
}

impl<SPI, CS> SdCardReader<SPI, CS>
where
    SPI: embedded_hal::spi::FullDuplex<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::spi::FullDuplex<u8>>::Error: core::fmt::Debug,
{
    pub fn new(spi: SPI, cs: CS) -> Result<Self, FileError> {
        let ts = DummyTimeSource {};
        let mut controller = sdmmc::Controller::new(sdmmc::SdMmcSpi::new(spi, cs), ts);
        controller
            .device()
            .init()
            .map_err(|e| sdmmc::Error::DeviceError(e))?;
        let volume = controller.get_volume(sdmmc::VolumeIdx(0))?;
        let root_dir = controller.open_root_dir(&volume)?;
        Ok(Self {
            controller,
            volume,
            root_dir,
        })
    }

    pub fn open_intro(&mut self) -> Result<DataReader, FileError> {
        let file = self.controller.open_file_in_dir(
            &mut self.volume,
            &self.root_dir,
            INTRO_FILE_NAME,
            sdmmc::Mode::ReadOnly,
        )?;
        let file_name = sdmmc::ShortFileName::create_from_str(INTRO_FILE_NAME)
            .map_err(|e| FileError::FilenameError(e))?;
        Ok(DataReader::new(file, file_name))
    }
}

impl<SPI, CS> FileReader for SdCardReader<SPI, CS>
where
    SPI: embedded_hal::spi::FullDuplex<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
    <SPI as embedded_hal::spi::FullDuplex<u8>>::Error: core::fmt::Debug,
{
    fn read_data(&mut self, file: &mut sdmmc::File, out: &mut [u8]) -> Result<usize, FileError> {
        self.controller.read(&self.volume, file, out)
    }
}

#[derive(Debug)]
pub enum PlayError {
    FileError(FileError),
    NoValidMp3Frame,
}

pub struct Mp3Player<'a> {
    read_index: usize,
    read_end: usize,
    write_index: usize,
    last_frame_index: usize,
    mp3_data: &'a mut [u8; MP3_DATA_LENGTH],
    decoder: mp3::Decoder<'a>,
    last_frame_rate: Option<time::Hertz>,
    pcm_buffer: &'a mut [i16; mp3::MAX_SAMPLES_PER_FRAME],
    current_song: Option<DataReader>,
}

impl<'a> Mp3Player<'a> {
    pub fn new(
        mp3_data: &'a mut [u8; MP3_DATA_LENGTH],
        decoder_data: &'a mut mp3::DecoderData,
        pcm_buffer: &'a mut [i16; mp3::MAX_SAMPLES_PER_FRAME],
    ) -> Self {
        Self {
            read_index: 0,
            read_end: 0,
            write_index: 0,
            last_frame_index: 0,
            mp3_data,
            decoder: mp3::Decoder::new(decoder_data),
            pcm_buffer: pcm_buffer,
            last_frame_rate: None,
            current_song: None,
        }
    }

    pub fn play_song(
        &mut self,
        data_reader: DataReader,
        file_reader: &mut impl FileReader,
        sound_device: &mut SoundDevice,
    ) -> Result<(), PlayError> {
        self.current_song.replace(data_reader);
        self.init_buffer(file_reader)
            .map_err(|e| PlayError::FileError(e))?;
        let last_frame_rate = self.last_frame_rate.ok_or(PlayError::NoValidMp3Frame)?;
        sound_device.start_playing(self, file_reader, last_frame_rate)
    }

    pub fn fill_buffer(&mut self, file_reader: &mut impl FileReader) -> Result<(), FileError> {
        self.fill_buffer_intern(file_reader, MP3_MAX_READ)
    }

    fn init_buffer(&mut self, file_reader: &mut impl FileReader) -> Result<(), FileError> {
        self.fill_buffer_intern(file_reader, MP3_DATA_LENGTH)
    }

    pub fn fill_buffer_intern(
        &mut self,
        file_reader: &mut impl FileReader,
        max_read: usize,
    ) -> Result<(), FileError> {
        if !self.prepare_read() {
            return Ok(());
        }
        let data_reader = self
            .current_song
            .as_mut()
            .expect("prepare_read to have returned");
        let write_end = if self.write_index < self.read_index {
            self.read_index
        } else {
            MP3_DATA_LENGTH
        };
        let to_read = max_read.min(write_end - self.write_index);
        let out_slice = &mut self.mp3_data[self.write_index..to_read + self.write_index];
        let bytes_red = data_reader.read_data(file_reader, out_slice)?;
        if bytes_red < to_read {
            self.current_song.take();
        }
        self.set_last_frame(bytes_red + self.write_index);
        if self.read_end < self.write_index {
            self.read_end = self.last_frame_index;
        }
        self.write_index += bytes_red;
        if self.write_index == MP3_DATA_LENGTH {
            self.write_index = 0;
        }
        Ok(())
    }

    fn prepare_read(&mut self) -> bool {
        if self.current_song.is_none() {
            return false;
        }
        if self.write_index < self.read_index && self.read_index - self.write_index < MP3_MAX_READ {
            return false; //not enough space to read
        }
        if self.write_index == 0 && self.read_index >= MP3_MAX_READ {
            let uncomplete_len = MP3_DATA_LENGTH - self.last_frame_index;
            let (left, right) = self.mp3_data.split_at_mut(self.last_frame_index);
            left[0..uncomplete_len].copy_from_slice(right);
            self.write_index = uncomplete_len;
            self.last_frame_index = 0;
        } //else (self.read_index < self.write_index) can read
        return true;
    }

    fn set_last_frame(&mut self, end_index: usize) {
        loop {
            let (begin, end) = (self.last_frame_index, end_index);
            let frame_size = self.next_frame_size(begin, end);
            if let Some(bytes) = frame_size {
                if bytes + self.last_frame_index > end_index {
                    break;
                }
                self.last_frame_index += bytes;
            } else {
                break;
            }
        }
    }

    fn next_frame_size(&mut self, mp3_data_begin: usize, mp3_data_end: usize) -> Option<usize> {
        let mp3_buffer = &self.mp3_data[mp3_data_begin..mp3_data_end];
        let peek = self.decoder.peek(mp3_buffer);
        match peek {
            mp3::DecodeResult::Successful(bytes, _) | mp3::DecodeResult::SkippedData(bytes) => {
                Some(bytes)
            }
            _ => None,
        }
    }

    pub fn next_frame(&mut self, dma_buffer: &mut [u16]) -> usize {
        fn advance_read_index(
            offset: usize,
            read_index: &mut usize,
            read_end: &mut usize,
            write_index: usize,
        ) {
            *read_index += offset;
            if write_index < *read_index && *read_index == *read_end {
                *read_index = 0;
                *read_end = write_index;
            }
        }

        let mp3: &[u8] = &self.mp3_data[self.read_index..self.read_end];
        if mp3.len() == 0 {
            return 0;
        }
        let pcm_buffer = &mut self.pcm_buffer;
        let decode_result = self.decoder.decode(mp3, pcm_buffer);
        match decode_result {
            mp3::DecodeResult::Successful(bytes_read, frame) => {
                advance_read_index(
                    bytes_read,
                    &mut self.read_index,
                    &mut self.read_end,
                    self.write_index,
                );
                let freq = frame.sample_rate;
                self.last_frame_rate.replace(freq.hz());
                let samples = pcm_buffer
                    .iter()
                    .step_by(frame.channels as usize)
                    .take(frame.sample_count as usize)
                    .map(|sample| ((*sample as i32) - (core::i16::MIN as i32)) as u16 >> 4);
                let mut index: usize = 0;
                for val in samples {
                    dma_buffer[index] = val;
                    index += 1;
                }
                index
            }
            mp3::DecodeResult::SkippedData(skipped_data) => {
                advance_read_index(
                    skipped_data,
                    &mut self.read_index,
                    &mut self.read_end,
                    self.write_index,
                );
                self.next_frame(dma_buffer)
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

    pub fn start_playing(
        &mut self,
        mp3_player: &mut Mp3Player,
        file_reader: &mut impl FileReader,
        freq: time::Hertz,
    ) -> Result<(), PlayError> {
        self.fill_pcm_buffer(0, mp3_player, file_reader)?;
        self.fill_pcm_buffer(1, mp3_player, file_reader)?;
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
            mp3_player
                .fill_buffer(file_reader)
                .map_err(|e| PlayError::FileError(e))
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

type SpiPins = (
    hal::gpio::gpioa::PA5<AF5>,
    hal::gpio::gpioa::PA6<AF5>,
    hal::gpio::gpioa::PA7<AF5>,
);
type SpiType = Spi<hal::stm32::SPI1, SpiPins>;

fn init_spi(
    spi1: hal::stm32::SPI1,
    sck: hal::gpio::gpioa::PA5<Input<Floating>>,
    miso: hal::gpio::gpioa::PA6<Input<Floating>>,
    mosi: hal::gpio::gpioa::PA7<Input<Floating>>,
    moder: &mut gpioa::MODER,
    afrl: &mut gpioa::AFRL,
    apb2: &mut hal::rcc::APB2,
    clocks: &hal::rcc::Clocks,
    freq: impl Into<hal::time::Hertz>,
) -> SpiType {
    let sck = sck.into_af5(moder, afrl);
    let miso = miso.into_af5(moder, afrl);
    let mosi = mosi.into_af5(moder, afrl);
    let spi_mode = embedded_hal::spi::Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    let spi = Spi::spi1(spi1, (sck, miso, mosi), spi_mode, freq, *clocks, apb2);
    spi
}

fn apb1enr() -> &'static stm32f303::rcc::APB1ENR {
    unsafe { &(*stm32f303::RCC::ptr()).apb1enr }
}

fn ahbenr() -> &'static stm32f303::rcc::AHBENR {
    unsafe { &(*stm32f303::RCC::ptr()).ahbenr }
}

fn enable_dma2_ch3_interrupt() {
    unsafe { stm32f303::NVIC::unmask(Interrupt::DMA2_CH3) };
}

#[app(device = stm32f3xx_hal::stm32, monotonic = rtfm::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        sound_device: SoundDevice<'static>,
        mp3_player: Mp3Player<'static>,
        card_reader: SdCardReader<SpiType, hal::gpio::PXx<Output<PushPull>>>,
        pa4: gpioa::PA4<Analog>,
    }
    #[init(spawn=[play_intro])]
    fn init(cx: init::Context) -> init::LateResources {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        let device = cx.device;

        let flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let clocks = init_clocks(rcc.cfgr, flash);

        let logger: &Logger<InterruptSync> = unsafe {
            LOGGER.replace(Logger {
                inner: InterruptSync::new(destination::itm::Itm::new(core.ITM)),
                level: log::LevelFilter::Trace,
            });
            LOGGER.as_ref().unwrap()
        };
        cortex_m_log::log::init(logger).expect("To set logger");

        let mut gpioa = device.GPIOA.split(&mut rcc.ahb);
        let pa4 = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

        let cs = gpioa
            .pa1
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
            .downgrade()
            .downgrade();

        let spi = init_spi(
            device.SPI1,
            gpioa.pa5,
            gpioa.pa6,
            gpioa.pa7,
            &mut gpioa.moder,
            &mut gpioa.afrl,
            &mut rcc.apb2,
            &clocks,
            18.mhz(),
        );

        let card_reader = SdCardReader::new(spi, cs)
            .map_err(|e| {
                error!("Card reader init failed: {:?}", e);
                e
            })
            .expect("To have a card reader");

        let buffers = unsafe { &mut BUFFERS };
        let ccram_buffers = unsafe { &mut CCRAM_BUFFERS };
        let mp3_player = Mp3Player::new(
            &mut buffers.mp3_data,
            &mut ccram_buffers.mp3_decoder_data,
            &mut buffers.pcm_buffer,
        );

        let sound_device = SoundDevice::new(
            &mut buffers.dma_buffer,
            clocks.sysclk(),
            device.TIM2,
            device.DAC,
            device.DMA2,
        );

        info!("Init finished");

        init::LateResources {
            sound_device,
            mp3_player,
            card_reader,
            pa4,
        }
    }

    #[task(resources=[card_reader, sound_device, mp3_player])]
    fn play_intro(cx: play_intro::Context) {
        let mut card_reader = cx.resources.card_reader;
        let mut sound_device = cx.resources.sound_device;
        let mut mp3_player = cx.resources.mp3_player;
        let play_result = card_reader.lock(|card_reader| {
            sound_device.lock(|sound_device| {
                mp3_player.lock(|mp3_player| {
                    card_reader
                        .open_intro()
                        .map_err(|e| PlayError::FileError(e))
                        .and_then(|song| {
                            let file_name = song.file_name();
                            mp3_player.play_song(song, card_reader, sound_device)?;
                            Ok(file_name)
                        })
                })
            })
        });
        match play_result {
            Ok(file_name) => info!("Playing {}", file_name),
            Err(e) => error!("Intro file can't be played: {:?}", e),
        }
    }

    #[idle(resources=[sound_device])]
    fn idle(mut cx: idle::Context) -> ! {
        static mut TICK: Option<Instant> = None;
        static mut ALREADY_STOPPED: bool = false;
        TICK.replace(Instant::now());
        info!("idle");
        loop {
            if TICK.unwrap().elapsed().as_cycles() > 4_000_000 {
                *TICK = Some(Instant::now());
                let playing = cx.resources.sound_device.lock(|sd| sd.playing);
                if !playing && !*ALREADY_STOPPED {
                    info!("Music stopped");
                    *ALREADY_STOPPED = false
                }
            }
        }
    }

    #[task(capacity=1, priority=8, resources=[ sound_device, mp3_player, card_reader])]
    fn process_dma_request(cx: process_dma_request::Context, new_state: DmaState) {
        match new_state {
            DmaState::Unknown => panic!("Unknown dma state"),
            DmaState::Error => {
                error!("Dma request error. Stop playing");
                cx.resources.sound_device.stop_playing();
            }
            DmaState::HalfTrigger | DmaState::TriggerComplete => {
                let index = if new_state == DmaState::HalfTrigger {
                    0
                } else {
                    1
                };
                let fil_res = cx.resources.sound_device.fill_pcm_buffer(
                    index,
                    cx.resources.mp3_player,
                    cx.resources.card_reader,
                );
                if fil_res.is_err() {
                    cx.resources.sound_device.stop_playing();
                }
            }
        }
    }

    #[task(binds = DMA2_CH3, priority=8, resources=[sound_device], spawn=[process_dma_request])]
    fn dma2_ch3(cx: dma2_ch3::Context) {
        let state = cx.resources.sound_device.dma_interrupt();
        if cx.spawn.process_dma_request(state).is_err() {
            cx.resources.sound_device.stop_playing();
            error!("Spawn error. Stop playing!");
        }
    }
    extern "C" {
        fn EXTI0();
        fn EXTI1();
    }
};
