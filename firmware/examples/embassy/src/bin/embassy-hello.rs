#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embassy_zynq7000::{Config, InterruptConfig, L2CacheMode, LevelShifterConfig};
use embassy_zynq7000::{clocks, gpio, log as embassy_log, uart};
use embedded_io::Write;
use log::{LevelFilter, error, info};

use zynq7000_rt as _;

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
    uart.write_all(b"-- Zynq 7000 Embassy Hello World --\n\r")
        .unwrap();
    // Safety: We are not multi-threaded yet.
    let (tx, _rx) = uart.split();
    unsafe { embassy_log::uart_blocking::init_unsafe_single_core(tx, LevelFilter::Trace, false) };

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut led = gpio::Output::new(p.MIO7, gpio::PinState::Low);
    loop {
        info!("Hello, world!");
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
