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
use leds::{Leds, LedsState};
use mp3_player::{Mp3Player, PlayError};
use playlist::{Playlist, PlaylistMoveDirection, PlaylistName};
use sound_device::{DmaState, SoundDevice, DMA_LENGTH};

use core::time;
use cortex_m_log::destination;
use cortex_m_log::log::Logger;
use cortex_m_log::printer::itm::InterruptSync;
use embedded_hal::digital::v1_compat;
use hal::gpio::{
    gpioa, gpiob, Alternate, Analog, Floating, Input, OpenDrain, Output, PullDown, PushPull, AF5,
};
use hal::hal as embedded_hal;
use hal::spi::Spi;
use hal::time::Hertz;
use stm32l4xx_hal as hal;
use stm32l4xx_hal::prelude::*;

use log::{debug, error, info};
use mfrc522::{self, Mfrc522};
use rtic::app;
use rtic::cyccnt::U32Ext;

static mut LOGGER: Option<Logger<InterruptSync>> = None;
const LOG_LEVEL: log::LevelFilter = log::LevelFilter::Debug;
const MP3_DATA_LENGTH: usize = 10 * 1024;
const USER_CYCLIC_TIME: time::Duration = time::Duration::from_millis(125);
const CARD_SCAN_PAUSE: time::Duration = time::Duration::from_millis(1000);
const BTN_CLICK_DEBASE: time::Duration = time::Duration::from_millis(100);

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

    pub fn to_cycles(&self, duration: time::Duration) -> rtic::cyccnt::Duration {
        let s_part = (duration.as_secs() as u32) * self.frequency.0;
        let mms_part = (duration.subsec_micros() / 1000) * (self.frequency.0 / 1000);
        (s_part + mms_part).cycles()
    }
}

fn init_clocks(
    cfgr: hal::rcc::CFGR,
    mut flash: hal::flash::Parts,
    mut pwr: hal::pwr::Pwr,
) -> hal::rcc::Clocks {
    cfgr.hse(
        8.mhz(),
        hal::rcc::CrystalBypass::Disable,
        hal::rcc::ClockSecuritySystem::Disable,
    )
    .sysclk(72.mhz())
    .hclk(72.mhz())
    .pclk2(72.mhz())
    .pclk1(36.mhz())
    .freeze(&mut flash.acr, &mut pwr)
}

type Spi1Pins = (
    gpioa::PA5<Alternate<AF5, Input<Floating>>>,
    gpioa::PA6<Alternate<AF5, Input<Floating>>>,
    gpioa::PA7<Alternate<AF5, Input<Floating>>>,
);

type Spi2Pins = (
    gpiob::PB13<Alternate<AF5, Input<Floating>>>,
    gpiob::PB14<Alternate<AF5, Input<Floating>>>,
    gpiob::PB15<Alternate<AF5, Input<Floating>>>,
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
        polarity: embedded_hal::spi::Polarity::IdleLow,
        phase: embedded_hal::spi::Phase::CaptureOnFirstTransition,
    };

    let spi = Spi::spi1(spi1, (sck, miso, mosi), spi_mode, freq, *clocks, apb2);
    spi
}

type RFIDReaderType = Mfrc522<Spi2Type, v1_compat::OldOutputPin<gpioa::PA8<Output<PushPull>>>>;

fn init_spi2(
    spi2: hal::stm32::SPI2,
    sck: gpiob::PB13<Input<Floating>>,
    miso: gpiob::PB14<Input<Floating>>,
    mosi: gpiob::PB15<Input<Floating>>,
    moder: &mut gpiob::MODER,
    afrh: &mut gpiob::AFRH,
    apb1: &mut hal::rcc::APB1R1,
    clocks: &hal::rcc::Clocks,
    freq: impl Into<Hertz>,
) -> Spi2Type {
    let sck = sck.into_af5(moder, afrh);
    let miso = miso.into_af5(moder, afrh);
    let mosi = mosi.into_af5(moder, afrh);
    let spi_mode = embedded_hal::spi::Mode {
        polarity: embedded_hal::spi::Polarity::IdleLow,
        phase: embedded_hal::spi::Phase::CaptureOnFirstTransition,
    };

    let spi = Spi::spi2(spi2, (sck, miso, mosi), spi_mode, freq, *clocks, apb1);
    spi
}

fn init_rfid_reader(spi2: Spi2Type, cs2: gpioa::PA8<Output<PushPull>>) -> RFIDReaderType {
    let rfid_reader =
        Mfrc522::new(spi2, v1_compat::OldOutputPin::new(cs2)).and_then(|mut rfid_reader| {
            let version = rfid_reader.version()?;
            info!("RFID reader version: 0x{:0x}", version);
            Ok((rfid_reader, version))
        });
    match rfid_reader {
        Ok((rfid_reader, version)) => {
            if version != 0x92 && version != 0x93 {
                panic!("Wrong RFID version {}", version);
            }
            rfid_reader
        }
        Err(e) => panic!("Can't initialize RFID reader, error: {:?}", e),
    }
}
fn config_exti(exti: hal::stm32::EXTI, syscfg: &hal::stm32::SYSCFG) -> hal::stm32::EXTI {
    syscfg.exticr1.modify(|_, w| unsafe {
        w.exti0().bits(0b000);
        w.exti1().bits(0b000);
        w.exti3().bits(0b000);
        w
    });
    exti.imr1.modify(|_, w| {
        w.mr0().set_bit();
        w.mr1().set_bit();
        w.mr3().set_bit();
        w
    });
    exti.rtsr1.modify(|_, w| {
        w.tr0().set_bit();
        w.tr1().set_bit();
        w.tr3().set_bit();
        w
    });
    exti.ftsr1.modify(|_, w| {
        w.tr0().clear_bit();
        w.tr1().clear_bit();
        w.tr3().clear_bit();
        w
    });
    exti
}

pub struct PlayingResources {
    pub sound_device: SoundDevice<'static>,
    pub mp3_player: Mp3Player<'static>,
    pub card_reader: SdCardReader<hal::gpio::gpioa::PA3<Output<OpenDrain>>>,
}

pub enum ButtonKind {
    Previous,
    Pause,
    Next,
}

impl ButtonKind {
    pub fn new(interrupt: hal::stm32::Interrupt) -> Self {
        use hal::stm32::Interrupt;
        match interrupt {
            Interrupt::EXTI0 => ButtonKind::Next,
            Interrupt::EXTI1 => ButtonKind::Previous,
            Interrupt::EXTI3 => ButtonKind::Pause,
            _ => panic!("Interrupt not mappable to a button"),
        }
    }
}

pub struct Buttons {
    pub button_next: gpiob::PB2<Input<PullDown>>,
    pub button_prev: gpiob::PB12<Input<PullDown>>,
    pub button_pause: gpiob::PB10<Input<PullDown>>,
}

impl Buttons {
    pub fn new(
        prev: gpiob::PB12<Input<Floating>>,
        pause: gpiob::PB10<Input<Floating>>,
        next: gpiob::PB2<Input<Floating>>,
        moder: &mut gpiob::MODER,
        pupdr: &mut gpiob::PUPDR,
    ) -> Self {
        let button_next = next.into_pull_down_input(moder, pupdr);
        let button_prev = prev.into_pull_down_input(moder, pupdr);
        let button_pause = pause.into_pull_down_input(moder, pupdr);

        Buttons {
            button_next,
            button_prev,
            button_pause,
        }
    }
}

fn rfid_read_card(rfid_reader: &mut RFIDReaderType) -> Option<mfrc522::Uid> {
    let atqa = rfid_reader.reqa();
    atqa.and_then(|atqa| rfid_reader.select(&atqa)).ok()
}

#[app(device = stm32l4xx_hal::stm32, monotonic = rtic::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        playing_resources: PlayingResources,
        pa4: gpioa::PA4<Analog>,
        #[init(None)]
        current_playlist: Option<Playlist>,
        buttons: Buttons,
        rfid_reader: RFIDReaderType,
        leds: Leds,
        exti: hal::stm32::EXTI,
        btn_click_debase: rtic::cyccnt::Duration,
        card_scan_pause: rtic::cyccnt::Duration,
        user_cyclic_time: rtic::cyccnt::Duration,
    }

    #[init(spawn=[start_playlist, user_cyclic])]
    fn init(cx: init::Context) -> init::LateResources {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        let device = cx.device;

        let mut rcc = device.RCC.constrain();
        let flash = device.FLASH.constrain();
        let pwr = device.PWR.constrain(&mut rcc.apb1r1);
        let clocks = init_clocks(rcc.cfgr, flash, pwr);

        let logger: &Logger<InterruptSync> = unsafe {
            LOGGER.replace(Logger {
                inner: InterruptSync::new(destination::itm::Itm::new(core.ITM)),
                level: LOG_LEVEL,
            });
            LOGGER.as_ref().expect("to have a logger")
        };
        cortex_m_log::log::init(logger).expect("To set logger");
        debug!("Configuring");

        let mut gpiob = device.GPIOB.split(&mut rcc.ahb2);
        let buttons = Buttons::new(
            gpiob.pb12,
            gpiob.pb10,
            gpiob.pb2,
            &mut gpiob.moder,
            &mut gpiob.pupdr,
        );

        let mut gpioa = device.GPIOA.split(&mut rcc.ahb2);
        let pa4 = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr); // Speaker out

        let cs1 = gpioa
            .pa3
            .into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);

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

        debug!("Initializing sd-card reader");
        let card_reader = SdCardReader::new(spi1, cs1, 8.mhz(), clocks)
            .map_err(|e| {
                error!("Card reader init failed: {:?}", e);
                e
            })
            .expect("To have a sd card reader");

        let cs2 = gpioa
            .pa8
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

        let spi2 = init_spi2(
            device.SPI2,
            gpiob.pb13,
            gpiob.pb14,
            gpiob.pb15,
            &mut gpiob.moder,
            &mut gpiob.afrh,
            &mut rcc.apb1r1,
            &clocks,
            400.khz(),
        );

        debug!("Initializing rfid reader");
        let rfid_reader = init_rfid_reader(spi2, cs2);

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

        debug!("Initializing sound device");
        let sound_device = SoundDevice::new(
            &mut buffers.dma_buffer,
            clocks.sysclk(),
            device.TIM2,
            device.DAC1,
            device.DMA2,
        );

        let leds = Leds::new(device.GPIOE.split(&mut rcc.ahb2));
        let exti = config_exti(device.EXTI, &device.SYSCFG);

        debug!("Start cyclic task");
        cx.spawn
            //.start_playlist(PlaylistName::from_bytes("87B13133"))
            //.start_playlist(PlaylistName::from_bytes("F460482A"))
            .start_playlist(PlaylistName::from_bytes("02".as_bytes()))
            .expect("To start playlist");
        cx.spawn.user_cyclic().expect("To start cyclic task");

        let time_computer = CyclesComputer::new(clocks.sysclk());
        let btn_click_debase = time_computer.to_cycles(BTN_CLICK_DEBASE);
        let card_scan_pause = time_computer.to_cycles(CARD_SCAN_PAUSE);
        let user_cyclic_time = time_computer.to_cycles(USER_CYCLIC_TIME);

        info!("Init finished");
        init::LateResources {
            playing_resources: PlayingResources {
                sound_device,
                mp3_player,
                card_reader,
            },
            pa4,
            buttons,
            rfid_reader,
            leds,
            exti,
            btn_click_debase,
            card_scan_pause,
            user_cyclic_time,
        }
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        info!("idle");
        loop {}
    }

    #[task(resources=[playing_resources, user_cyclic_time, rfid_reader], priority=8, spawn=[start_playlist], schedule=[user_cyclic])]
    fn user_cyclic(cx: user_cyclic::Context) {
        let mut uid_hex: [u8; 8] = [0; 8];
        if let Some(uid) = rfid_read_card(cx.resources.rfid_reader) {
            hex::encode_to_slice(&uid.bytes()[0..4], &mut uid_hex[..]);
            let next_playlist = PlaylistName::from_bytes(&uid_hex[..]);
            cx.spawn.start_playlist(next_playlist).ok();
        }
        let user_cyclic_time = *cx.resources.user_cyclic_time;
        cx.schedule
            .user_cyclic(cx.scheduled + user_cyclic_time)
            .expect("To be able to schedule user_cyclic");
    }

    #[task(resources=[playing_resources], priority=8, spawn=[playlist_next])]
    fn btn_pressed(cx: btn_pressed::Context, btn_kind: ButtonKind) {
        debug!("In btn_pressed");
        match btn_kind {
            ButtonKind::Next => {
                cx.resources.playing_resources.sound_device.stop_playing();
                info!("Trigger playlist next");
                cx.spawn.playlist_next(true).ok();
            }
            ButtonKind::Previous => {
                cx.resources.playing_resources.sound_device.stop_playing();
                info!("Trigger playlist prev");
                cx.spawn.playlist_next(false).ok();
            }
            ButtonKind::Pause => {
                info!("Toggle pause");
                cx.resources.playing_resources.sound_device.toggle_pause();
            }
        }
    }

    #[task(resources=[playing_resources, current_playlist, card_scan_pause, leds], spawn=[playlist_next])]
    fn start_playlist(cx: start_playlist::Context, directory_name: PlaylistName) {
        let start_playlistResources {
            mut playing_resources,
            mut current_playlist,
            card_scan_pause,
            leds,
        } = cx.resources;
        let mut play_successful = false;
        current_playlist.lock(|playlist| {
            playing_resources.lock(|pr| {
                let same_playlist = playlist
                    .as_ref()
                    .map(|p| directory_name == p.name())
                    .unwrap_or(false);
                let at_begining = pr
                    .sound_device
                    .play_pause_elapsed()
                    .map(|elapsed| elapsed <= *card_scan_pause)
                    .unwrap_or(false);
                if same_playlist && at_begining {
                    info!("Same card");
                } else {
                    info!("Starting playlist: {}", directory_name);
                    pr.sound_device.stop_playing();
                    playlist
                        .take()
                        .map(|playlist| playlist.close(&mut pr.card_reader));
                    let play_result = pr
                        .card_reader
                        .open_directory(&directory_name.as_str())
                        .map_err(|e| {
                            error!("Can not open directory: {}, err: {:?}", directory_name, e);
                            leds.set_state(LedsState::Red, true);
                            e
                        })
                        .map(|next_playlist| {
                            playlist.replace(next_playlist);
                            leds.set_state(LedsState::Green, true);
                        });
                    play_successful = play_result.is_ok()
                }
            })
        });
        if play_successful {
            cx.spawn.playlist_next(true).ok();
        }
    }

    #[task(resources=[playing_resources, current_playlist, leds])]
    fn playlist_next(mut cx: playlist_next::Context, forward: bool) {
        info!("Playlist_next called forward: {}", forward);
        let direction = if forward {
            PlaylistMoveDirection::Next
        } else {
            PlaylistMoveDirection::Previous
        };
        let leds = &mut cx.resources.leds;
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
                            Ok(Some(file_name)) => {
                                leds.set_state(LedsState::Green, true);
                                info!("Playing {}", file_name);
                            }
                            Ok(None) => {
                                leds.set_state(LedsState::Yellow, true);
                                info!("Playing nothing");
                            }
                            Err(e) => {
                                leds.set_state(LedsState::Red, true);
                                error!("Playlist {} can't be played: {:?}", playlist.name(), e);
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

    #[task(binds = EXTI0, resources=[btn_click_debase, exti], schedule=[btn_pressed])]
    fn exti0(cx: exti0::Context) {
        debug!("in exti0");
        cx.resources.exti.pr1.modify(|_, w| w.pr0().set_bit());
        let sched_next = cx.start + *cx.resources.btn_click_debase;
        cx.schedule
            .btn_pressed(sched_next, ButtonKind::new(hal::stm32::Interrupt::EXTI0))
            .ok();
    }

    #[task(binds = EXTI1, resources=[btn_click_debase, exti], schedule=[btn_pressed])]
    fn exti1(cx: exti1::Context) {
        cx.resources.exti.pr1.modify(|_, w| w.pr1().set_bit());
        let sched_next = cx.start + *cx.resources.btn_click_debase;
        cx.schedule
            .btn_pressed(sched_next, ButtonKind::new(hal::stm32::Interrupt::EXTI1))
            .ok();
    }

    #[task(binds = EXTI3, resources=[btn_click_debase, exti], schedule=[btn_pressed])]
    fn exti3(cx: exti3::Context) {
        cx.resources.exti.pr1.modify(|_, w| w.pr2().set_bit());
        let sched_next = cx.start + *cx.resources.btn_click_debase;
        cx.schedule
            .btn_pressed(sched_next, ButtonKind::new(hal::stm32::Interrupt::EXTI3))
            .ok();
    }

    extern "C" {
        fn I2C3_EV();
        fn I2C3_ER();
    }
};
