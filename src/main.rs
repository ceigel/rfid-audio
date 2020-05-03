#![no_std]
#![no_main]

extern crate embedded_mp3 as mp3;
use panic_itm as _;

mod data_reader;
mod hex;
mod leds;
mod mp3_player;
mod playlist;
mod sound_device;
use data_reader::SdCardReader;
use playlist::{Playlist, PlaylistMoveDirection, PlaylistName};
use sound_device::{DmaState, SoundDevice, DMA_LENGTH};

use crate::hal::time::Hertz;
use core::time::Duration;
use cortex_m_log::destination;
use cortex_m_log::log::Logger;
use cortex_m_log::printer::itm::InterruptSync;
use embedded_hal::digital::{v1_compat, v2::InputPin};
use hal::gpio::{gpioa, gpiob, Analog, Floating, Input, Output, PullDown, PushPull, AF5};
use hal::hal as embedded_hal;
use hal::spi::{Phase, Polarity, Spi};
use leds::{Leds, LedsState};
use log::{debug, error, info};
use mfrc522::{self, Mfrc522};
use mp3_player::{Mp3Player, PlayError};
use rtfm::app;
use rtfm::cyccnt::U32Ext;
use stm32f3xx_hal as hal;
use stm32f3xx_hal::prelude::*;

static mut LOGGER: Option<Logger<InterruptSync>> = None;
const LOG_LEVEL: log::LevelFilter = log::LevelFilter::Debug;
const MP3_DATA_LENGTH: usize = 10 * 1024;
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

type Spi1Pins = (
    hal::gpio::gpioa::PA5<AF5>,
    hal::gpio::gpioa::PA6<AF5>,
    hal::gpio::gpioa::PA7<AF5>,
);

type Spi2Pins = (
    hal::gpio::gpiob::PB13<AF5>,
    hal::gpio::gpiob::PB14<AF5>,
    hal::gpio::gpiob::PB15<AF5>,
);
pub(crate) type Spi1Type = Spi<hal::stm32::SPI1, Spi1Pins>;
pub(crate) type Spi2Type = Spi<hal::stm32::SPI2, Spi2Pins>;

fn init_spi1(
    spi1: hal::stm32::SPI1,
    sck: gpioa::PA5<Input<Floating>>,
    miso: gpioa::PA6<Input<Floating>>,
    mosi: gpioa::PA7<Input<Floating>>,
    moder: &mut gpioa::MODER,
    afrl: &mut gpioa::AFRL,
    apb2: &mut hal::rcc::APB2,
    clocks: &hal::rcc::Clocks,
    freq: impl Into<Hertz>,
) -> Spi1Type {
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

type RFIDReaderType = Mfrc522<Spi2Type, v1_compat::OldOutputPin<hal::gpio::PXx<Output<PushPull>>>>;

fn init_rfid_reader(
    spi2: hal::stm32::SPI2,
    mut gpiob: gpiob::Parts,
    apb1: &mut hal::rcc::APB1,
    clocks: &hal::rcc::Clocks,
) -> RFIDReaderType {
    let cs2 = gpiob
        .pb12
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
        .downgrade()
        .downgrade();
    let spi2 = init_spi2(
        spi2,
        gpiob.pb13,
        gpiob.pb14,
        gpiob.pb15,
        &mut gpiob.moder,
        &mut gpiob.afrh,
        apb1,
        &clocks,
        400.khz(),
    );
    let rfid_reader =
        Mfrc522::new(spi2, v1_compat::OldOutputPin::new(cs2)).and_then(|mut rfid_reader| {
            let version = rfid_reader.version()?;
            info!("RFID reader version: 0x{:0x}", version);
            Ok(rfid_reader)
        });
    match rfid_reader {
        Ok(rfid_reader) => rfid_reader,
        Err(e) => panic!("Can't initialize RFID reader, error: {:?}", e),
    }
}

fn init_spi2(
    spi2: hal::stm32::SPI2,
    sck: gpiob::PB13<Input<Floating>>,
    miso: gpiob::PB14<Input<Floating>>,
    mosi: gpiob::PB15<Input<Floating>>,
    moder: &mut gpiob::MODER,
    afrh: &mut gpiob::AFRH,
    apb1: &mut hal::rcc::APB1,
    clocks: &hal::rcc::Clocks,
    freq: impl Into<Hertz>,
) -> Spi2Type {
    let sck = sck.into_af5(moder, afrh);
    let miso = miso.into_af5(moder, afrh);
    let mosi = mosi.into_af5(moder, afrh);
    let spi_mode = embedded_hal::spi::Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    let spi = Spi::spi2(spi2, (sck, miso, mosi), spi_mode, freq, *clocks, apb1);
    spi
}

pub struct PlayingResources {
    pub sound_device: SoundDevice<'static>,
    pub mp3_player: Mp3Player<'static>,
    pub card_reader: SdCardReader<hal::gpio::PXx<Output<PushPull>>>,
}

pub struct Buttons {
    pub button_next: gpioa::PAx<Input<PullDown>>,
    pub button_prev: gpioa::PAx<Input<PullDown>>,
    pub button_pause: gpioa::PAx<Input<PullDown>>,
    pub btn_next_processed: bool,
    pub btn_prev_processed: bool,
    pub btn_pause_processed: bool,
}

impl Buttons {
    pub fn new(
        pa0: gpioa::PA0<Input<Floating>>,
        pa1: gpioa::PA1<Input<Floating>>,
        pa3: gpioa::PA3<Input<Floating>>,
        moder: &mut gpioa::MODER,
        pupdr: &mut gpioa::PUPDR,
    ) -> Self {
        let button_next = pa0.into_pull_down_input(moder, pupdr).downgrade();
        let button_prev = pa1.into_pull_down_input(moder, pupdr).downgrade();
        let button_pause = pa3.into_pull_down_input(moder, pupdr).downgrade();

        Buttons {
            button_next,
            button_prev,
            button_pause,
            btn_next_processed: false,
            btn_prev_processed: false,
            btn_pause_processed: false,
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

fn rfid_read_card(rfid_reader: &mut RFIDReaderType) -> Option<mfrc522::Uid> {
    let atqa = rfid_reader.reqa();
    atqa.and_then(|atqa| rfid_reader.select(&atqa)).ok()
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
        rfid_reader: Mfrc522<Spi2Type, v1_compat::OldOutputPin<hal::gpio::PXx<Output<PushPull>>>>,
        //leds: Leds,
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
            LOGGER.as_ref().expect("to have a logger")
        };
        cortex_m_log::log::init(logger).expect("To set logger");

        let mut gpioa = device.GPIOA.split(&mut rcc.ahb);
        let buttons = Buttons::new(
            gpioa.pa0,
            gpioa.pa1,
            gpioa.pa3,
            &mut gpioa.moder,
            &mut gpioa.pupdr,
        );

        let pa4 = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr); // Speaker out

        let cs1 = gpioa
            .pa2
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
            .downgrade()
            .downgrade();

        let spi1 = init_spi1(
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
        let card_reader = SdCardReader::new(spi1, cs1, 8.mhz(), clocks)
            .map_err(|e| {
                error!("Card reader init failed: {:?}", e);
                e
            })
            .expect("To have a sd card reader");

        let gpiob = device.GPIOB.split(&mut rcc.ahb);
        let rfid_reader = init_rfid_reader(device.SPI2, gpiob, &mut rcc.apb1, &clocks);

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

        let leds = Leds::new(device.GPIOE.split(&mut rcc.ahb));

        /*cx.spawn
        //.start_playlist(PlaylistName::new("87B13133"))
        .start_playlist(PlaylistName::new("F460482A"))
        //.start_playlist(PlaylistName::new("02"))
        .expect("To start playlist");*/
        cx.spawn.user_cyclic().expect("To start cyclic task");
        info!("Init finished");
        init::LateResources {
            playing_resources: PlayingResources {
                sound_device,
                mp3_player,
                card_reader,
            },
            pa4,
            time_computer,
            buttons,
            rfid_reader,
            //leds,
        }
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        info!("idle");
        loop {}
    }

    #[task(resources=[playing_resources, buttons, time_computer, rfid_reader], priority=8, spawn=[playlist_next, start_playlist], schedule=[user_cyclic])]
    fn user_cyclic(cx: user_cyclic::Context) {
        let Buttons {
            button_next,
            button_prev,
            button_pause,
            btn_next_processed,
            btn_prev_processed,
            btn_pause_processed,
        } = cx.resources.buttons;

        let next_trigger = should_trigger(button_next, btn_next_processed);
        let prev_trigger = should_trigger(button_prev, btn_prev_processed);
        let pause_trigger = should_trigger(button_pause, btn_pause_processed);
        if next_trigger || prev_trigger {
            cx.resources.playing_resources.sound_device.stop_playing();
            if next_trigger {
                info!("Trigger playlist next");
                cx.spawn.playlist_next(true).ok();
            }
            if prev_trigger {
                info!("Trigger playlist prev");
                cx.spawn.playlist_next(false).ok();
            }
        }
        if pause_trigger {
            info!("Trigger pause");
            cx.resources.playing_resources.sound_device.toggle_pause();
        }
        let mut uid_hex: [u8; 8] = [0; 8];
        if let Some(uid) = rfid_read_card(cx.resources.rfid_reader) {
            hex::encode_to_slice(&uid.bytes()[0..4], &mut uid_hex[..]);
            let next_playlist = PlaylistName::from_bytes(&uid_hex[..]);
            cx.spawn.start_playlist(next_playlist).ok();
        }
        let user_cyclic: rtfm::cyccnt::Duration =
            cx.resources.time_computer.to_cycles(USER_CYCLIC_TIME);
        cx.schedule
            .user_cyclic(cx.scheduled + user_cyclic)
            .expect("To be able to schedule user_cyclic");
    }

    #[task(resources=[playing_resources, current_playlist], spawn=[playlist_next])]
    fn start_playlist(cx: start_playlist::Context, directory_name: PlaylistName) {
        let start_playlistResources {
            mut playing_resources,
            mut current_playlist,
        } = cx.resources;
        let play_result = current_playlist.lock(|playlist| {
            playing_resources.lock(|pr| {
                pr.sound_device.stop_playing();
                playlist
                    .take()
                    .map(|playlist| playlist.close(&mut pr.card_reader));
                pr.card_reader
                    .open_directory(&directory_name.as_str())
                    .map_err(|e| {
                        error!("Can not open directory: {:?}, err: {:?}", directory_name, e);
                        //leds.set_state(LedsState::Red, true);
                        e
                    })
                    .map(|next_playlist| {
                        info!("Starting playlist: {}", next_playlist.name());
                        playlist.replace(next_playlist);
                        //leds.set_state(LedsState::Green, true);
                    })
            })
        });
        if play_result.is_ok() {
            cx.spawn
                .playlist_next(true)
                .expect("to spawn playlist next");
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
        let playing_resources = &mut cx.resources.playing_resources;
        let current_playlist = &mut cx.resources.current_playlist;
        current_playlist.lock(|playlist| {
            playing_resources.lock(|pr| {
                pr.sound_device.stop_playing();
                match playlist {
                    None => info!("Playlist none"),
                    Some(playlist) => {
                        let play_result = playlist
                            .move_next(direction, &mut pr.card_reader)
                            .map_err(|e| PlayError::from(e))
                            .and_then(|song| {
                                if let Some(song) = song {
                                    let file_name = song.file_name();
                                    pr.mp3_player
                                        .play_song(song, &mut pr.card_reader, &mut pr.sound_device)
                                        .map(|_| Some(file_name))
                                } else {
                                    Ok(None)
                                }
                            });
                        match play_result {
                            Ok(Some(file_name)) => info!("Playing {}", file_name),
                            Ok(None) => info!("Playing nothing"),
                            Err(e) => {
                                error!("Playlist {} can't be played: {:?}", playlist.name(), e)
                            }
                        }
                    }
                }
            })
        });
    }

    #[task(capacity=1, priority=8, resources=[playing_resources, current_playlist], spawn=[playlist_next])]
    fn process_dma_request(cx: process_dma_request::Context, new_state: DmaState) {
        let pr = cx.resources.playing_resources;
        let playlist = cx
            .resources
            .current_playlist
            .as_mut()
            .expect("to have a playlist to play");
        let current_song = playlist.current_song().expect("to play a song");
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
                    current_song,
                    &mut pr.card_reader,
                ) {
                    Err(_) => {
                        cx.spawn.playlist_next(true).ok();
                    }
                    _ => {}
                }
            }
            DmaState::Stopped => {
                cx.spawn.playlist_next(true).ok();
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
