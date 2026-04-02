//! Example which uses the UART1 to send log messages.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embassy_zynq7000::{
    Config as EmbassyConfig, InterruptConfig, L2CacheMode, LevelShifterConfig, bind_interrupts,
};
use embassy_zynq7000::{
    clocks,
    gpio::{Output, PinState},
    log as embassy_log, uart,
};
use log::{LevelFilter, error, info};

use zynq7000_rt as _;

bind_interrupts!(struct Irqs {
    Uart1 => embassy_zynq7000::uart::InterruptHandler<embassy_zynq7000::peripherals::UART1>;
});

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_zynq7000::init(EmbassyConfig {
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
    let uart = uart::Uart::new(
        p.UART1,
        p.MIO48,
        p.MIO49,
        Irqs,
        uart::Config::new_with_clk_config(uart_clk_config),
    );
    let (mut logger, _rx) = uart.split();
    embedded_io::Write::write_all(&mut logger, b"-- Zynq 7000 Logging example --\n\r").unwrap();
    embedded_io::Write::flush(&mut logger).unwrap();

    embassy_log::rb::init(LevelFilter::Trace);

    let led = Output::new(p.MIO7, PinState::Low);
    spawner.spawn(led_task(led).unwrap());
    let mut log_buf: [u8; 2048] = [0; 2048];
    let frame_queue = embassy_log::rb::get_frame_queue();
    loop {
        let next_frame_len = frame_queue.receive().await;
        embassy_log::rb::read_next_frame(next_frame_len, &mut log_buf);
        logger.write(&log_buf[0..next_frame_len]).await;
    }
}

#[embassy_executor::task]
async fn led_task(mut mio_led: Output<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        mio_led.toggle();
        info!("Toggling LED");
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
