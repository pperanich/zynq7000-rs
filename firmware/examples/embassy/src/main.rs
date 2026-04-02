#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embassy_zynq7000::gpio::{self, Output};
use embassy_zynq7000::{Config, InterruptConfig, L2CacheMode, LevelShifterConfig};
use log::error;

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_zynq7000::init(Config {
        ps_clock_frequency: zedboard_bsp::PS_CLOCK_FREQUENCY,
        l2_cache_mode: L2CacheMode::Initialize,
        level_shifter_config: Some(LevelShifterConfig::EnableAll),
        interrupt_config: Some(InterruptConfig::AllInterruptsToCpu0),
    })
    .unwrap();
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut led = Output::new(p.MIO7, gpio::PinState::Low);
    loop {
        led.toggle();
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
