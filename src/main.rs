#![no_std]
#![no_main]

extern crate panic_semihosting;

use cortex_m::iprintln;
use cortex_m::peripheral::ITM;
use rtfm::app;
use stm32f3xx_hal::stm32 as stm32f303;
use stm32f3xx_hal::{prelude::*, stm32, time::Hertz};

mod wav_data;
const DMA_LENGTH: usize = 64;
static mut DMA_BUFFER: [u16; DMA_LENGTH] = [0; DMA_LENGTH];

pub fn init_tim2(freq: Hertz, clocks: &stm32f3xx_hal::rcc::Clocks, tim2: &stm32f303::TIM2) {
    let apb1enr = unsafe { &(*stm32f303::RCC::ptr()).apb1enr };
    apb1enr.modify(|_, w| w.tim2en().set_bit());
    let sysclk = clocks.sysclk();
    let arr = sysclk.0 / freq.0;
    tim2.cr2.write(|w| w.mms().update());
    tim2.arr.write(|w| w.arr().bits(arr));
    tim2.cr1.modify(|_, w| w.cen().enabled());
}

fn init_dac1(mut gpioa: stm32f3xx_hal::gpio::gpioa::Parts, dac: &stm32f303::DAC) {
    let apb1enr = unsafe { &(*stm32f303::RCC::ptr()).apb1enr };
    let _pa4 = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    apb1enr.modify(|_, w| w.dac1en().set_bit());
    dac.cr.write(|w| {
        w.boff1().disabled();
        w.ten1().enabled();
        w.tsel1().tim2_trgo();
        w
    });
    dac.cr.modify(|_, w| w.en1().enabled());
}

pub fn init_dma2(dma_buffer: &[u16], dac: &stm32f303::DAC, dma2: &stm32f303::DMA2) {
    let ahbenr = unsafe { &(*stm32f303::RCC::ptr()).ahbenr };
    ahbenr.modify(|_, w| w.dma2en().set_bit());
    let ma = dma_buffer.as_ptr() as usize as u32;
    let pa = &dac.dhr12r1 as *const stm32f303::dac::DHR12R1 as usize as u32;
    let ndt = dma_buffer.len() as u16;

    dma2.ch3.mar.write(|w| w.ma().bits(ma));
    dma2.ch3.par.write(|w| w.pa().bits(pa));
    dma2.ch3.ndtr.write(|w| w.ndt().bits(ndt));
    dma2.ch3.cr.write(|w| {
        w.dir().from_memory(); // source is memory
        w.mem2mem().disabled(); // disable memory to memory transfer
        w.minc().enabled(); // increment memory address every transfer
        w.pinc().disabled(); // don't increment peripheral address every transfer
        w.msize().bits16(); // memory word size is 32 bits
        w.psize().bits16(); // peripheral word size is 32 bits
                            //w.circ().enabled(); // dma mode is circular
        w.pl().high(); // set dma priority to high
        w.teie().enabled(); // trigger an interrupt if an error occurs
                            // w.tcie().enabled(); // trigger an interrupt when transfer is complete
                            //w.htie().enabled() // trigger an interrupt when half the transfer is complete
        w
    });
    unsafe { stm32f303::NVIC::unmask(stm32f303::Interrupt::DMA2_CH3) };
    dac.cr.modify(|_, w| w.dmaen1().enabled());
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

/*
fn linearly_interpolate(wt: &[u16], index: f32) -> u16 {
    let int_part: usize = index as usize;
    let frac_part: f32 = index - int_part as f32;
    let y0 = wt[int_part] as f32;
    let y1 = wt[(int_part + 1) % wt.len()] as f32;
    (y0 + ((y1 - y0) * frac_part)) as u16
}

fn audio_callback(phase: &mut f32, buffer: &mut [u16], length: usize, offset: usize) {
    let wt_sin = wavetable::SIN;
    let wt_length = wt_sin.len();
    let dx = 261.6 * (1. / 44100.);

    for t in 0..length {
        let wt_index = *phase * wt_length as f32;
        let channel_1 = linearly_interpolate(&wt_sin, wt_index) as u16;

        let frame = t + (offset * length);
        buffer[frame] = channel_1;

        *phase += dx;
        if *phase >= 1.0 {
            *phase = -1.0;
        }
    }
}
*/

#[app(device = stm32f3xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        itm: ITM,
        dma_buffer: &'static [u16],
        dma2: stm32::DMA2,
        dac: stm32::DAC,
        #[init(0.0)]
        phase: f32,
        mp3: &'static [u8],
    }
    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        // Initialize (enable) the monotonic timer (CYCCNT)
        let mut core = cx.core;
        let itm = core.ITM;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        let device: stm32::Peripherals = cx.device;

        let flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let clocks = init_clocks(rcc.cfgr, flash);
        init_tim2(wav_data::SAMPLE_RATE.hz(), &clocks, &device.TIM2);
        let gpioa = device.GPIOA.split(&mut rcc.ahb);
        init_dac1(gpioa, &device.DAC);
        let dma_buffer: &[u16] = &wav_data::WAV_TABLE;
        // let dma_buffer = unsafe { &mut DMA_BUFFER };
        init_dma2(dma_buffer, &device.DAC, &device.DMA2);
        device.DMA2.ch3.cr.modify(|_, w| w.en().enabled());

        let mp3 = include_bytes!("Glass.mp3");
        init::LateResources {
            itm,
            dma_buffer,
            dma2: device.DMA2,
            dac: device.DAC,
            mp3: mp3,
        }
    }

    #[idle(resources=[itm])]
    fn idle(mut cx: idle::Context) -> ! {
        cx.resources.itm.lock(|itm| {
            iprintln!(&mut itm.stim[0], "idle");
        });
        loop {}
    }

    #[task(binds = DMA2_CH3, priority=2, resources=[itm, dma_buffer, dma2, dac, phase])]
    fn dma2_ch3(cx: dma2_ch3::Context) {
        enum State {
            HT,
            TC,
            Error,
            Unknown,
        };

        // determine dma state
        let state = {
            let dma2 = cx.resources.dma2;

            // cache interrupt status before clearing interrupt flag
            let isr = dma2.isr.read();

            // clear interrupt flag and return dma state
            if isr.tcif3().is_complete() {
                dma2.ifcr.write(|w| w.ctcif3().clear());
                State::TC
            } else if isr.htif3().is_half() {
                dma2.ifcr.write(|w| w.chtif3().clear());
                State::HT
            } else if isr.teif3().is_error() {
                dma2.ifcr.write(|w| w.cteif3().clear());
                State::Error
            } else {
                State::Unknown
            }
        };

        /*let dma_buffer = cx.resources.dma_buffer;
        let buffer_len = dma_buffer.len();
        let mut phase: f32 = *cx.resources.phase;
        // invoke audio callback
        match state {
            State::HT => audio_callback(&mut phase, dma_buffer, buffer_len / 2, 0),
            State::TC => audio_callback(&mut phase, dma_buffer, buffer_len / 2, 1),
            _ => (),
        }
        *cx.resources.phase = phase;
        */
    }
};
