#![no_std]
#![no_main]

extern crate embedded_mp3 as mp3;
use panic_itm as _;

mod data_reader;
mod mp3_player;
mod playlist;
mod sound_device;
use data_reader::SdCardReader;
use playlist::{Playlist, PlaylistMoveDirection};
use sound_device::{DmaState, SoundDevice, DMA_LENGTH};

use crate::hal::time::Hertz;
use core::time::Duration;
use cortex_m_log::destination;
use cortex_m_log::log::Logger;
use cortex_m_log::printer::itm::InterruptSync;
use embedded_hal::digital::v2::InputPin;
use hal::gpio::{gpioa, Analog, Floating, Input, Output, PullDown, PushPull, AF5};
use hal::hal as embedded_hal;
use hal::spi::{Phase, Polarity, Spi};
use log::{debug, error, info};
use mp3_player::{Mp3Player, PlayError};
use rtfm::app;
use rtfm::cyccnt::{Instant, U32Ext};
use stm32f3xx_hal as hal;
use stm32f3xx_hal::prelude::*;

static mut LOGGER: Option<Logger<InterruptSync>> = None;
const LOG_LEVEL: log::LevelFilter = log::LevelFilter::Info;
const MP3_DATA_LENGTH: usize = 12 * 1024;
const USER_CYCLIC_TIME: Duration = Duration::from_millis(125);

struct CCRamBuffers {
    pub mp3_decoder_data: mp3::DecoderData,
}

struct Buffers {
    pub dma_buffer: [u16; DMA_LENGTH],
    pub mp3_data: [u8; MP3_DATA_LENGTH],
    pub pcm_buffer: [i16; mp3::MAX_SAMPLES_PER_FRAME],
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
            mp3_data: [0; MP3_DATA_LENGTH],
            pcm_buffer: [0; mp3::MAX_SAMPLES_PER_FRAME],
        }
    }
}

static mut BUFFERS: Buffers = Buffers::new();
#[link_section = ".ccram_data"]
static mut CCRAM_BUFFERS: CCRamBuffers = CCRamBuffers::new();

pub struct CyclesComputer {
    frequency: Hertz,
}

impl CyclesComputer {
    pub fn new(frequency: Hertz) -> Self {
        CyclesComputer { frequency }
    }

    pub fn to_cycles(&self, duration: Duration) -> rtfm::cyccnt::Duration {
        let s_part = (duration.as_secs() as u32) * self.frequency.0;
        let mms_part = (duration.subsec_micros() / 1000) * (self.frequency.0 / 1000);
        (s_part + mms_part).cycles()
    }
}

fn init_clocks(
    cfgr: stm32f3xx_hal::rcc::CFGR,
    mut flash: stm32f3xx_hal::flash::Parts,
) -> hal::rcc::Clocks {
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

pub(crate) type SpiType = Spi<hal::stm32::SPI1, SpiPins>;

fn init_spi(
    spi1: hal::stm32::SPI1,
    sck: gpioa::PA5<Input<Floating>>,
    miso: gpioa::PA6<Input<Floating>>,
    mosi: gpioa::PA7<Input<Floating>>,
    moder: &mut gpioa::MODER,
    afrl: &mut gpioa::AFRL,
    apb2: &mut hal::rcc::APB2,
    clocks: &hal::rcc::Clocks,
    freq: impl Into<Hertz>,
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

pub struct PlayingResources {
    pub sound_device: SoundDevice<'static>,
    pub mp3_player: Mp3Player<'static>,
    pub card_reader: SdCardReader<hal::gpio::PXx<Output<PushPull>>>,
}

#[derive(Copy, Clone)]
pub struct Pressed {
    time: Instant,
    seen_count: u32,
}

impl Pressed {
    pub fn new() -> Self {
        Self {
            time: Instant::now(),
            seen_count: 0,
        }
    }

    pub fn imcrement_seen(&mut self) {
        self.seen_count += 1;
    }

    pub fn elapsed(&self) -> rtfm::cyccnt::Duration {
        self.time.elapsed()
    }

    pub fn seen(&self) -> u32 {
        self.seen_count
    }
}

pub struct Buttons {
    pub button_next: gpioa::PAx<Input<PullDown>>,
    pub button_prev: gpioa::PAx<Input<PullDown>>,
    pub btn_next_processed: bool,
    pub btn_prev_processed: bool,
}

impl Buttons {
    pub fn new(
        button_next: gpioa::PAx<Input<PullDown>>,
        button_prev: gpioa::PAx<Input<PullDown>>,
    ) -> Self {
        Buttons {
            button_next,
            button_prev,
            btn_next_processed: false,
            btn_prev_processed: false,
        }
    }
}

fn should_trigger(btn: &impl InputPin, processed: &mut bool) -> bool {
    if btn.is_high().unwrap_or(false) {
        if !*processed {
            *processed = true;
            return true;
        }
    } else if *processed {
        *processed = false;
    }
    false
}

#[app(device = stm32f3xx_hal::stm32, monotonic = rtfm::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        playing_resources: PlayingResources,
        pa4: gpioa::PA4<Analog>,
        #[init(None)]
        current_playlist: Option<Playlist>,
        time_computer: CyclesComputer,
        buttons: Buttons,
    }

    #[init(spawn=[start_playlist, user_cyclic])]
    fn init(cx: init::Context) -> init::LateResources {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        let device = cx.device;

        let flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let clocks = init_clocks(rcc.cfgr, flash);
        let time_computer = CyclesComputer::new(clocks.sysclk());

        let logger: &Logger<InterruptSync> = unsafe {
            LOGGER.replace(Logger {
                inner: InterruptSync::new(destination::itm::Itm::new(core.ITM)),
                level: LOG_LEVEL,
            });
            LOGGER.as_ref().unwrap()
        };
        cortex_m_log::log::init(logger).expect("To set logger");

        let mut gpioa = device.GPIOA.split(&mut rcc.ahb);
        let pa4 = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

        let cs = gpioa
            .pa2
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
            .downgrade()
            .downgrade();

        let button_next = gpioa
            .pa0
            .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr)
            .downgrade();

        let button_prev = gpioa
            .pa1
            .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr)
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
            400.khz(),
        );

        let card_reader = SdCardReader::new(spi, cs, 16.mhz(), clocks)
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
        debug!(
            "Size of buffers: ccram_buffers: {}, buffers: {}",
            core::mem::size_of::<CCRamBuffers>(),
            core::mem::size_of::<Buffers>()
        );

        let sound_device = SoundDevice::new(
            &mut buffers.dma_buffer,
            clocks.sysclk(),
            device.TIM2,
            device.DAC,
            device.DMA2,
        );

        info!("Init finished");

        cx.spawn.start_playlist("02").expect("To start play_intro");
        cx.spawn.user_cyclic().expect("To start cyclic task");
        init::LateResources {
            playing_resources: PlayingResources {
                sound_device,
                mp3_player,
                card_reader,
            },
            pa4,
            time_computer,
            buttons: Buttons::new(button_next, button_prev),
        }
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        info!("idle");
        loop {}
    }

    #[task(resources=[playing_resources, buttons, time_computer], priority=8, spawn=[playlist_next], schedule=[user_cyclic])]
    fn user_cyclic(cx: user_cyclic::Context) {
        let Buttons {
            button_next,
            button_prev,
            btn_next_processed,
            btn_prev_processed,
        } = cx.resources.buttons;

        if should_trigger(button_next, btn_next_processed) {
            cx.resources.playing_resources.sound_device.stop_playing();
            info!("Trigger playlist next");
            cx.spawn.playlist_next(true).ok();
        }
        if should_trigger(button_prev, btn_prev_processed) {
            cx.resources.playing_resources.sound_device.stop_playing();
            info!("Trigger playlist prev");
            cx.spawn.playlist_next(false).ok();
        }
        let user_cyclic: rtfm::cyccnt::Duration =
            cx.resources.time_computer.to_cycles(USER_CYCLIC_TIME);
        cx.schedule
            .user_cyclic(cx.scheduled + user_cyclic)
            .expect("To be able to schedule user_cyclic");
    }

    #[task(resources=[playing_resources, current_playlist], spawn=[playlist_next])]
    fn start_playlist(mut cx: start_playlist::Context, directory_name: &'static str) {
        let currently_playing = cx.resources.current_playlist.is_some();
        let next_playlist = cx.resources.playing_resources.lock(|pr| {
            if currently_playing {
                pr.sound_device.stop_playing();
            }
            pr.card_reader.open_directory(directory_name)
        });
        match next_playlist {
            Ok(playlist) => {
                info!("Starting playlist: {}", playlist.name());
                if cx.resources.current_playlist.replace(playlist).is_none() {
                    cx.spawn
                        .playlist_next(true)
                        .expect("to spawn playlist next");
                }
            }
            Err(e) => {
                error!("Can not open directory: {}, err: {:?}", directory_name, e);
            }
        }
    }

    #[task(resources=[playing_resources, current_playlist])]
    fn playlist_next(mut cx: playlist_next::Context, forward: bool) {
        info!("Playlist_next called forward: {}", forward);
        let direction = if forward {
            PlaylistMoveDirection::Next
        } else {
            PlaylistMoveDirection::Previous
        };
        if let Some(playlist) = cx.resources.current_playlist {
            let play_result = cx.resources.playing_resources.lock(|pr| {
                playlist
                    .move_next(direction, &mut pr.card_reader)
                    .map_err(|e| PlayError::from(e))
                    .and_then(|song| match song {
                        None => Ok(None),
                        Some(song) => {
                            let file_name = song.file_name();
                            pr.mp3_player.play_song(
                                song,
                                &mut pr.card_reader,
                                &mut pr.sound_device,
                            )?;
                            Ok(Some(file_name))
                        }
                    })
            });
            match play_result {
                Ok(file_name) => info!("Playing {:?}", file_name),
                Err(e) => error!("Playlist {} can't be played: {:?}", playlist.name(), e),
            }
        }
    }

    #[task(resources=[playing_resources])]
    fn play_intro(mut cx: play_intro::Context) {
        let play_result = cx.resources.playing_resources.lock(|pr| {
            let card_reader = &mut pr.card_reader;
            let sound_device = &mut pr.sound_device;
            let mp3_player = &mut pr.mp3_player;
            card_reader
                .open_file("intro.mp3")
                .map_err(|e| PlayError::FileError(e))
                .and_then(|song| {
                    let file_name = song.file_name();
                    mp3_player.play_song(song, card_reader, sound_device)?;
                    Ok(file_name)
                })
        });
        match play_result {
            Ok(file_name) => info!("Playing {}", file_name),
            Err(e) => error!("Intro file can't be played: {:?}", e),
        }
    }

    #[task(capacity=1, priority=8, resources=[playing_resources], spawn=[playlist_next])]
    fn process_dma_request(cx: process_dma_request::Context, new_state: DmaState) {
        let pr = cx.resources.playing_resources;
        match new_state {
            DmaState::Unknown => panic!("Unknown dma state"),
            DmaState::HalfTrigger | DmaState::TriggerComplete => {
                let index = if new_state == DmaState::HalfTrigger {
                    0
                } else {
                    1
                };
                match pr.sound_device.fill_pcm_buffer(
                    index,
                    &mut pr.mp3_player,
                    &mut pr.card_reader,
                ) {
                    Err(_) => {
                        cx.spawn.playlist_next(true).unwrap();
                    }
                    _ => {}
                }
            }
            DmaState::Stopped => {
                cx.spawn.playlist_next(true).unwrap();
            }
            _ => {}
        }
    }

    #[task(binds = DMA2_CH3, priority=8, resources=[playing_resources], spawn=[process_dma_request])]
    fn dma2_ch3(cx: dma2_ch3::Context) {
        let pr = cx.resources.playing_resources;
        let state = pr.sound_device.dma_interrupt();
        if cx.spawn.process_dma_request(state).is_err() {
            error!("Spawn error. Stop playing!");
        }
    }

    extern "C" {
        fn EXTI0();
        fn EXTI1();
    }
};
