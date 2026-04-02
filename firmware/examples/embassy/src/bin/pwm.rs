//! PWM example which uses a PWM pin routed through EMIO.
//!
//! This example puts the PWM output on the EMIO channel of TTC0 channel 0.
//!
//! On the Zedboard, the PWM waveform output will be on the W12 pin of PMOD JB1. The Zedboard
//! reference FPGA design must be flashed onto the Zedboard for this to work.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embassy_zynq7000::{Config as EmbassyConfig, InterruptConfig, L2CacheMode, LevelShifterConfig};
use embassy_zynq7000::{
    clocks,
    gpio::{Output, PinState},
    log as embassy_log, ttc, uart,
};
use embedded_io::Write;
use fugit::RateExtU32;
use log::{LevelFilter, error, info};
use zynq7000_rt as _;

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_zynq7000::init(EmbassyConfig {
        ps_clock_frequency: zedboard_bsp::PS_CLOCK_FREQUENCY,
        l2_cache_mode: L2CacheMode::Initialize,
        level_shifter_config: Some(LevelShifterConfig::EnableAll),
        interrupt_config: Some(InterruptConfig::AllInterruptsToCpu0),
    })
    .unwrap();

    // Unwrap is okay, the address is definitely valid.
    let ttc_0 = ttc::Ttc::new(p.TTC0);
    let mut pwm = ttc::Pwm::new_with_cpu_clk(ttc_0.ch0, clocks::get(), 1000.Hz()).unwrap();
    pwm.set_duty_cycle_percent(50).unwrap();

    // Set up the UART, we are logging with it.
    let uart_clk_config =
        uart::ClockConfig::new_autocalc_with_error(clocks::get().io_clocks(), 115200)
            .unwrap()
            .0;
    let mut uart = uart::Uart::new_blocking(
        p.UART1,
        p.MIO48,
        p.MIO49,
        uart::Config::new_with_clk_config(uart_clk_config),
    );
    uart.write_all(b"-- Zynq 7000 PWM example--\n\r").unwrap();
    // Safety: We are not multi-threaded yet.
    let (tx, _rx) = uart.split();
    unsafe { embassy_log::uart_blocking::init_unsafe_single_core(tx, LevelFilter::Trace, false) };

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut led = Output::new(p.MIO7, PinState::Low);
    let mut current_duty = 0;
    loop {
        led.toggle();

        pwm.set_duty_cycle_percent(current_duty).unwrap();
        info!("Setting duty cycle to {current_duty}%");
        current_duty += 5;
        if current_duty > 100 {
            current_duty = 0;
        }

        ticker.next().await;
    }
}

#[zynq7000_rt::exception(DataAbort)]
fn data_abort_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

#[zynq7000_rt::exception(Undefined)]
fn undefined_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

#[zynq7000_rt::exception(PrefetchAbort)]
fn prefetch_handler(_faulting_addr: usize) -> ! {
    loop {
        nop();
    }
}

/// Panic handler
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("Panic: {info:?}");
    loop {}
}
