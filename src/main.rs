#![no_std]
#![no_main]

extern crate embedded_mp3 as mp3;
use panic_itm as _;

mod data_reader;
mod mp3_player;
mod playlist;
mod sound_device;
use data_reader::SdCardReader;
use playlist::Playlist;
use sound_device::{DmaState, SoundDevice, DMA_LENGTH};

use cortex_m_log::destination;
use cortex_m_log::log::Logger;
use cortex_m_log::printer::itm::InterruptSync;
use hal::gpio::{gpioa, Analog, Floating, Input, Output, PushPull, AF5};
use hal::hal as embedded_hal;
use hal::spi::{Phase, Polarity, Spi};
use log::{debug, error, info};
use mp3_player::{Mp3Player, PlayError};
use rtfm::app;
use rtfm::cyccnt::Instant;
use stm32f3xx_hal as hal;
use stm32f3xx_hal::prelude::*;

static mut LOGGER: Option<Logger<InterruptSync>> = None;
const LOG_LEVEL: log::LevelFilter = log::LevelFilter::Debug;

const MP3_DATA_LENGTH: usize = 12 * 1024;

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

pub struct PlayingResources {
    pub sound_device: SoundDevice<'static>,
    pub mp3_player: Mp3Player<'static>,
    pub card_reader: SdCardReader<SpiType, hal::gpio::PXx<Output<PushPull>>>,
}

#[app(device = stm32f3xx_hal::stm32, monotonic = rtfm::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        playing_resources: PlayingResources,
        pa4: gpioa::PA4<Analog>,
        #[init(None)]
        current_playlist: Option<Playlist>,
    }
    #[init(spawn=[start_playlist])]
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
                level: LOG_LEVEL,
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
        debug!(
            "Size of buffers: ccram_buffers: {}, buffers: {}",
            core::mem::size_of::<CCRamBuffers>(),
            core::mem::size_of::<Buffers>()
        );

        let sound_device = SoundDevice::new(
            &mut buffers.dma_buffer,
            clocks.sysclk(),
            device.TIM2,
            device.TIM7,
            device.DAC,
            device.DMA2,
        );

        info!("Init finished");

        cx.spawn.start_playlist("02").expect("To start play_intro");
        init::LateResources {
            playing_resources: PlayingResources {
                sound_device,
                mp3_player,
                card_reader,
            },
            pa4,
        }
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
                if cx.resources.current_playlist.replace(playlist).is_none() {
                    cx.spawn.playlist_next().expect("to spawn playlist next");
                }
            }
            Err(e) => {
                error!("Can not open directory: {}, err: {:?}", directory_name, e);
            }
        }
    }

    #[task(resources=[playing_resources, current_playlist])]
    fn playlist_next(mut cx: playlist_next::Context) {
        if let Some(playlist) = cx.resources.current_playlist {
            let play_result = cx.resources.playing_resources.lock(|pr| {
                playlist
                    .move_next(&mut pr.card_reader)
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

    #[idle(resources=[playing_resources])]
    fn idle(mut cx: idle::Context) -> ! {
        static mut TICK: Option<Instant> = None;
        static mut ALREADY_STOPPED: bool = false;
        TICK.replace(Instant::now());
        info!("idle");
        loop {
            if TICK.unwrap().elapsed().as_cycles() > 4_000_000 {
                *TICK = Some(Instant::now());
                let stopping = cx
                    .resources
                    .playing_resources
                    .lock(|pr| pr.sound_device.stopping);
                if stopping && !*ALREADY_STOPPED {
                    info!("Music stopped");
                    log::logger().flush();
                    *ALREADY_STOPPED = true
                }
            }
        }
    }

    #[task(capacity=1, priority=8, resources=[playing_resources])]
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
                let fil_res =
                    pr.sound_device
                        .fill_pcm_buffer(index, &mut pr.mp3_player, &mut pr.card_reader);
                if fil_res.is_err() {
                    pr.sound_device.stop_playing();
                }
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

    #[task(binds = TIM7, priority=1, resources=[playing_resources], spawn=[playlist_next])]
    fn tim7(mut cx: tim7::Context) {
        debug!("Tim7 called, now: {:?}", Instant::now());
        cx.resources.playing_resources.lock(|pr| {
            pr.sound_device.playing_stop_timer_interrupt();
        });
        cx.spawn.playlist_next().unwrap();
    }

    extern "C" {
        fn EXTI0();
        fn EXTI1();
    }
};
