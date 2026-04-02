//! Open-drain pin mode examples
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Ticker};
use embassy_zynq7000::{Config as EmbassyConfig, InterruptConfig, L2CacheMode, LevelShifterConfig};
use embassy_zynq7000::{
    clocks,
    gpio::{Flex, Output, PinState},
    log as embassy_log, uart,
};
use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use log::{LevelFilter, error, info, warn};

/// Try to talk to a DHT22 sensor connected at MIO0.
const DHT22_AT_MIO0: bool = true;

/// Open drain pin testing. MIO9 needs to be tied to MIO14.
const OPEN_DRAIN_PINS_MIO9_TO_MIO14: bool = false;

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
    uart.write_all(b"-- Zynq 7000 DHT22 --\n\r").unwrap();

    // Safety: We are not multi-threaded yet.
    let (tx, _rx) = uart.split();
    unsafe { embassy_log::uart_blocking::init_unsafe_single_core(tx, LevelFilter::Trace, false) };

    let mut delay = Delay;

    let mut one_wire_pin = Flex::new(p.MIO0);
    one_wire_pin.configure_as_output_open_drain(PinState::High, true);

    if OPEN_DRAIN_PINS_MIO9_TO_MIO14 {
        let mut flex_pin_0 = Flex::new(p.MIO9);
        flex_pin_0.configure_as_output_open_drain(PinState::High, true);
        let mut flex_pin_1 = Flex::new(p.MIO14);
        flex_pin_1.configure_as_input_floating().unwrap();
        // Should be high because of pull up.
        info!(
            "Flex Pin 1 state (should be high): {}",
            flex_pin_1.is_high()
        );
        info!(
            "Flex Pin 0 state (should be high): {}",
            flex_pin_0.is_high()
        );
        flex_pin_0.set_low();
        info!("Flex Pin 1 state (should be low): {}", flex_pin_1.is_high());
        info!("Flex Pin 0 state (should be low): {}", flex_pin_0.is_high());
        flex_pin_0.set_high();
        delay.delay_us(5);
        info!(
            "Flex Pin 1 state (should be high): {}",
            flex_pin_1.is_high()
        );
        info!(
            "Flex Pin 0 state (should be high): {}",
            flex_pin_0.is_high()
        );

        flex_pin_1.configure_as_output_open_drain(PinState::Low, true);
        info!("Flex Pin 1 state (should be low): {}", flex_pin_1.is_high());
        info!("Flex Pin 0 state (should be low): {}", flex_pin_0.is_high());

        flex_pin_1.set_high();
        delay.delay_us(5);
        info!(
            "Flex Pin 1 state (should be high): {}",
            flex_pin_1.is_high()
        );
        info!(
            "Flex Pin 0 state (should be high): {}",
            flex_pin_0.is_high()
        );

        flex_pin_1.set_low();
        info!("Flex Pin 1 state (should be low): {}", flex_pin_1.is_high());
        info!("Flex Pin 0 state (should be low): {}", flex_pin_0.is_high());
    }

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut led = Output::new(p.MIO7, PinState::Low);
    loop {
        if DHT22_AT_MIO0 {
            let result = dht_sensor::dht22::r#async::read(&mut delay, &mut one_wire_pin).await;
            match result {
                Ok(reading) => {
                    info!("Temperature: {} C", reading.temperature);
                    info!("Humidity: {} %", reading.relative_humidity);
                }
                Err(err) => {
                    warn!("Reading error: {err:?}");
                }
            }
        }
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
