#![no_std]
#![no_main]

extern crate embedded_mp3 as mp3;
use panic_itm as _;

#[macro_use(block)]
extern crate nb;

mod battery_voltage;
mod buttons;
mod cycles_computer;
mod data_reader;
mod hex;
mod mp3_player;
mod playlist;
mod sleep_manager;
mod sound_device;
mod state;

use buttons::{ButtonKind, Buttons};
use data_reader::SdCardReader;
use mp3_player::{Mp3Player, PlayError};
use playlist::{Playlist, PlaylistMoveDirection, PlaylistName};
use sleep_manager::SleepManager;
use sound_device::{DmaState, SoundDevice, DMA_LENGTH};

use core::time;
use cortex_m_log::destination;
use cortex_m_log::log::Logger;
use cortex_m_log::printer::itm::InterruptSync;
use cycles_computer::CyclesComputer;
use embedded_hal::digital::v1_compat;
use hal::adc::ADC;
use hal::gpio::{
    gpioa, gpiob, Alternate, Analog, Floating, Input, OpenDrain, Output, PushPull, AF5,
};
use hal::hal as embedded_hal;
use hal::spi::Spi;
use hal::time::Hertz;
use stm32l4xx_hal as hal;
use stm32l4xx_hal::prelude::*;

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::timer::CountDown;
use log::{debug, error, info};
use mfrc522::{self, Mfrc522};
use rtic::app;
use stm32l4xx_hal::stm32 as stm32l431;

static mut LOGGER: Option<Logger<InterruptSync>> = None;
const LOG_LEVEL: log::LevelFilter = log::LevelFilter::Debug;
const MP3_DATA_LENGTH: usize = 18 * 1024;
const USER_CYCLIC_TIME: time::Duration = time::Duration::from_millis(125);
const SLEEP_TIME: time::Duration = time::Duration::from_secs(5 * 60);
const LEDS_CYCLIC_TIME: time::Duration = time::Duration::from_millis(40);
const BATTERY_READER_CYCLIC_TIME: time::Duration = time::Duration::from_secs(10);
const SHUTTING_DOWN_TIME: time::Duration = time::Duration::from_secs(5);
const CARD_SCAN_PAUSE: time::Duration = time::Duration::from_millis(5000);
const BTN_CLICK_DEBASE: time::Duration = time::Duration::from_millis(500);
const BATTERY_SHUTDOWN_LEVEL: u16 = 3350;

fn gpioa_pupdr() -> &'static stm32l431::gpioa::PUPDR {
    unsafe { &(*stm32l431::GPIOA::ptr()).pupdr }
}

struct Buffers {
    pub dma_buffer: [u16; DMA_LENGTH],
    pub mp3_data: [u8; MP3_DATA_LENGTH],
    pub pcm_buffer: [i16; mp3::MAX_SAMPLES_PER_FRAME],
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
    .pll_source(hal::rcc::PllSource::HSE)
    .sysclk(72.mhz())
    .hclk(72.mhz())
    .pclk2(72.mhz())
    .pclk1(72.mhz())
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

pub struct PlayingResources {
    pub sound_device: SoundDevice<'static>,
    pub mp3_player: Mp3Player<'static>,
    pub card_reader: SdCardReader<hal::gpio::gpioa::PA3<Output<OpenDrain>>>,
    pub state_leds: state::StateLeds,
}

fn rfid_read_card(rfid_reader: &mut RFIDReaderType) -> Option<mfrc522::Uid> {
    let atqa = rfid_reader.reqa();
    atqa.and_then(|atqa| rfid_reader.select(&atqa)).ok()
}

fn print_cache_states() {
    let icache_state = if cortex_m::peripheral::SCB::icache_enabled() {
        "enabled"
    } else {
        "disabled"
    };
    let dcache_state = if cortex_m::peripheral::SCB::dcache_enabled() {
        "enabled"
    } else {
        "disabled"
    };
    debug!("ICache {}", icache_state);
    debug!("DCache {}", dcache_state);
}

pub struct DelayTimer<T: CountDown<Time = hal::time::Hertz>> {
    timer: T,
}

impl<T: CountDown<Time = hal::time::Hertz>> DelayTimer<T> {
    pub fn new(timer: T) -> Self {
        Self { timer }
    }
}

impl<T: CountDown<Time = hal::time::Hertz>> DelayUs<u32> for DelayTimer<T> {
    fn delay_us(&mut self, us: u32) {
        use hal::time::{MegaHertz, U32Ext};
        let mhz: MegaHertz = us.mhz();
        self.timer.start(mhz);
        block!(self.timer.wait()).expect("To be able to wait for timer");
    }
}

#[app(device = stm32l4xx_hal::stm32, monotonic = rtic::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        playing_resources: PlayingResources,
        audio_out: gpioa::PA4<Analog>,
        #[init(None)]
        current_playlist: Option<Playlist>,
        buttons: Buttons,
        rfid_reader: RFIDReaderType,
        sleep_manager: SleepManager,
        btn_click_debase: rtic::cyccnt::Duration,
        card_scan_pause: rtic::cyccnt::Duration,
        battery_reader: battery_voltage::BatteryReader,
        user_cyclic_time: rtic::cyccnt::Duration,
        leds_cyclic_time: rtic::cyccnt::Duration,
        time_computer: CyclesComputer,
    }

    #[init(spawn=[start_playlist, user_cyclic, leds_cyclic, battery_status])]
    fn init(cx: init::Context) -> init::LateResources {
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        core.SCB.set_sleepdeep();

        let device = cx.device;
        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.ahb2);

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
        let led_nose_b = gpioa
            .pa0
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
            .into_af1(&mut gpioa.moder, &mut gpioa.afrl);
        let led_nose_g = gpioa
            .pa1
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
            .into_af1(&mut gpioa.moder, &mut gpioa.afrl);
        let led_nose_r = gpioa
            .pa2
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
            .into_af1(&mut gpioa.moder, &mut gpioa.afrl);
        let led_eye = gpioa
            .pa11
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        let led_mouth = gpiob
            .pb11
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper)
            .into_af1(&mut gpiob.moder, &mut gpiob.afrh);
        let tim2_pwm = device.TIM2.pwm(
            (led_nose_b, led_nose_g, led_nose_r, led_mouth),
            1.khz(),
            clocks,
            &mut rcc.apb1r1,
        );
        let mut state_leds =
            state::StateLeds::new(tim2_pwm.0, tim2_pwm.1, tim2_pwm.2, led_eye, tim2_pwm.3);
        state_leds.set_state(state::State::Init(state::InitState::Begin));

        let spi1 = init_spi1(
            device.SPI1,
            gpioa.pa5,
            gpioa.pa6,
            gpioa.pa7,
            &mut gpioa.moder,
            &mut gpioa.afrl,
            &mut rcc.apb2,
            &clocks,
            250.khz(),
        );

        let spi2 = init_spi2(
            device.SPI2,
            gpiob.pb13,
            gpiob.pb14,
            gpiob.pb15,
            &mut gpiob.moder,
            &mut gpiob.afrh,
            &mut rcc.apb1r1,
            &clocks,
            250.khz(),
        );

        gpioa_pupdr().write(|w| w.pupdr6().pull_up().pupdr8().pull_up());

        let mut cs1 = gpioa
            .pa3
            .into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
        cs1.set_high().expect("To be able to set CS1 to high");
        let cs2 = gpioa
            .pa8
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

        state_leds.set_state(state::State::Init(state::InitState::SetSDCard));
        debug!("Initializing sd-card reader");
        let card_reader = SdCardReader::new(spi1, cs1, 8.mhz(), clocks)
            .map_err(|e| {
                error!("Card reader init failed: {:?}", e);
                e
            })
            .expect("To have a sd card reader");
        debug!("Initializing rfid reader");

        state_leds.set_state(state::State::Init(state::InitState::SetRFID));
        let rfid_reader = init_rfid_reader(spi2, cs2);

        let audio_out = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr); // Speaker out
        let mut audio_en = gpioa
            .pa12
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        audio_en.set_low().expect("To set audio_en low");

        let buffers = unsafe { &mut BUFFERS };
        let mp3_player = Mp3Player::new(&mut buffers.mp3_data, &mut buffers.pcm_buffer);
        debug!("Size of buffers: {}", core::mem::size_of::<Buffers>());

        debug!("Initializing sound device");
        let sound_device = SoundDevice::new(
            &mut buffers.dma_buffer,
            clocks.sysclk(),
            device.TIM6,
            device.DAC1,
            device.DMA1,
            audio_en,
        );

        debug!("Init ADC");
        let tim7 = hal::timer::Timer::tim7(device.TIM7, 1.mhz(), clocks, &mut rcc.apb1r1);
        let mut delay = DelayTimer::new(tim7);
        let adc = ADC::new(device.ADC, &mut rcc.ahb2, &mut rcc.ccipr, &mut delay);
        let adc_in = gpiob.pb0.into_analog(&mut gpiob.moder, &mut gpiob.pupdr);
        let battery_reader = battery_voltage::BatteryReader::new(adc, adc_in);

        debug!("Initializing buttons");
        let buttons = Buttons::new(
            gpiob.pb12,
            gpiob.pb10,
            gpiob.pb2,
            &mut gpiob.moder,
            &mut gpiob.pupdr,
        );

        debug!("Start cyclic task");
        cx.spawn
            .start_playlist(PlaylistName::from_bytes("49DFFE97".as_bytes())) // winamp intro
            //.start_playlist(PlaylistName::from_bytes("02".as_bytes())) // mozart stuff
            //.start_playlist(PlaylistName::from_bytes("87B13133".as_bytes())) // mama maria
            .expect("To start playlist");
        cx.spawn.user_cyclic().expect("To start user cyclic task");
        cx.spawn.leds_cyclic().expect("To start leds cyclic task");
        cx.spawn
            .battery_status()
            .expect("To start battery status task");

        let time_computer = CyclesComputer::new(clocks.sysclk());
        let btn_click_debase = time_computer.to_cycles(BTN_CLICK_DEBASE);
        let card_scan_pause = time_computer.to_cycles(CARD_SCAN_PAUSE);
        let user_cyclic_time = time_computer.to_cycles(USER_CYCLIC_TIME);
        let leds_cyclic_time = time_computer.to_cycles(LEDS_CYCLIC_TIME);
        let sleep_manager = SleepManager::new(SLEEP_TIME, USER_CYCLIC_TIME);

        print_cache_states();
        info!("Init finished");
        state_leds.set_state(state::State::Init(state::InitState::InitFinished));
        init::LateResources {
            playing_resources: PlayingResources {
                sound_device,
                mp3_player,
                card_reader,
                state_leds,
            },
            audio_out,
            buttons,
            rfid_reader,
            sleep_manager,
            btn_click_debase,
            card_scan_pause,
            battery_reader,
            user_cyclic_time,
            leds_cyclic_time,
            time_computer,
        }
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        info!("idle");
        loop {}
    }

    #[task(resources=[playing_resources, user_cyclic_time, rfid_reader, buttons, btn_click_debase, sleep_manager], priority=8, spawn=[start_playlist], schedule=[user_cyclic, btn_pressed])]
    fn user_cyclic(cx: user_cyclic::Context) {
        let mut uid_hex: [u8; 8] = [0; 8];
        if let Some(uid) = rfid_read_card(cx.resources.rfid_reader) {
            hex::encode_to_slice(&uid.bytes()[0..4], &mut uid_hex[..]);
            let next_playlist = PlaylistName::from_bytes(&uid_hex[..]);
            cx.spawn.start_playlist(next_playlist).ok();
        }

        let btns = cx.resources.buttons;
        let sched_next = cx.scheduled + *cx.resources.btn_click_debase;
        if btns.button_next.is_low().expect("To read button next") {
            cx.schedule.btn_pressed(sched_next, ButtonKind::Next).ok();
        } else if btns.button_prev.is_low().expect("To read button prev") {
            cx.schedule
                .btn_pressed(sched_next, ButtonKind::Previous)
                .ok();
        } else if btns.button_pause.is_low().expect("To read button pause") {
            cx.schedule.btn_pressed(sched_next, ButtonKind::Pause).ok();
        }

        let user_cyclic_time = *cx.resources.user_cyclic_time;
        let sound_device = &cx.resources.playing_resources.sound_device;
        cx.resources.sleep_manager.click(
            sound_device.is_playing(),
            &mut cx.resources.playing_resources.state_leds,
        );
        cx.schedule
            .user_cyclic(cx.scheduled + user_cyclic_time)
            .expect("To be able to schedule user_cyclic");
    }

    #[task(resources=[leds_cyclic_time, playing_resources], schedule=[leds_cyclic])]
    fn leds_cyclic(mut cx: leds_cyclic::Context) {
        let leds_cyclic_time = *cx.resources.leds_cyclic_time;
        cx.resources
            .playing_resources
            .lock(|pr| pr.state_leds.show_state());
        cx.schedule
            .leds_cyclic(cx.scheduled + leds_cyclic_time)
            .expect("To be able to schedule user_cyclic");
    }

    #[task(resources=[playing_resources], priority=8)]
    fn shut_down(cx: shut_down::Context) {
        let pr = cx.resources.playing_resources;
        SleepManager::shut_down(&mut pr.state_leds);
    }

    #[task(resources=[time_computer, battery_reader, playing_resources], schedule=[battery_status, shut_down])]
    fn battery_status(mut cx: battery_status::Context) {
        let start = rtic::cyccnt::Instant::now();
        let val = cx.resources.battery_reader.read();
        let tc = cx.resources.time_computer;
        let elapsed = tc.from_cycles(start.elapsed());
        let mut shut_down = false;
        info!("Battery value: {}, elapsed: {}us", val, elapsed.as_micros());
        cx.resources.playing_resources.lock(|pr| {
            if val < BATTERY_SHUTDOWN_LEVEL {
                pr.sound_device.stop_playing();
                pr.state_leds.set_state(state::State::ShuttingDown);
                shut_down = true;
            }
        });
        if shut_down {
            let shutting_down_time = tc.to_cycles(SHUTTING_DOWN_TIME);
            cx.schedule
                .shut_down(cx.scheduled + shutting_down_time)
                .expect("To be able to schedule shut_down");
            error!("Will sleep!");
        }
        let battery_reader_cyclic_time = tc.to_cycles(BATTERY_READER_CYCLIC_TIME);
        cx.schedule
            .battery_status(cx.scheduled + battery_reader_cyclic_time)
            .expect("To be able to schedule battery_status");
    }

    #[task(resources=[playing_resources], priority=8, spawn=[playlist_next])]
    fn btn_pressed(cx: btn_pressed::Context, btn_kind: ButtonKind) {
        debug!("In btn_pressed");
        match btn_kind {
            ButtonKind::Next => {
                info!("Stop playing: ");
                cx.resources.playing_resources.sound_device.stop_playing();
                info!("Trigger playlist next");
                cx.spawn.playlist_next(true).ok();
            }
            ButtonKind::Previous => {
                info!("Stop playing: ");
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

    #[task(resources=[playing_resources, current_playlist, card_scan_pause], spawn=[playlist_next])]
    fn start_playlist(cx: start_playlist::Context, directory_name: PlaylistName) {
        let start_playlistResources {
            mut playing_resources,
            mut current_playlist,
            card_scan_pause,
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
                    info!("Stop playing: ");
                    pr.sound_device.stop_playing();
                    info!("Starting playlist: {}", directory_name);
                    playlist
                        .take()
                        .map(|playlist| playlist.close(&mut pr.card_reader));
                    let play_result = pr
                        .card_reader
                        .open_directory(&directory_name.as_str())
                        .map_err(|e| {
                            error!("Can not open directory: {}, err: {:?}", directory_name, e);
                            pr.state_leds.set_state(state::State::Error);
                            e
                        })
                        .map(|next_playlist| {
                            playlist.replace(next_playlist);
                            pr.state_leds.set_state(state::State::Playing);
                        });
                    play_successful = play_result.is_ok()
                }
            })
        });
        if play_successful {
            cx.spawn.playlist_next(true).ok();
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
                info!("Stop playing: ");
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
                                pr.state_leds.set_state(state::State::Playing);
                                info!("Playing {}", file_name);
                            }
                            Ok(None) => {
                                pr.state_leds.set_state(state::State::NotPlaying);
                                info!("Playing nothing");
                            }
                            Err(e) => {
                                pr.state_leds.set_state(state::State::Error);
                                error!("Playlist {} can't be played: {:?}", playlist.name(), e);
                            }
                        };
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

    #[task(binds = DMA1_CH3, priority=8, resources=[playing_resources], spawn=[process_dma_request])]
    fn dma1_ch3(cx: dma1_ch3::Context) {
        let pr = cx.resources.playing_resources;
        let state = pr.sound_device.dma_interrupt();
        if cx.spawn.process_dma_request(state).is_err() {
            error!("Spawn error. Stop playing!");
        }
    }

    extern "C" {
        fn I2C3_EV();
        fn I2C3_ER();
    }
};
