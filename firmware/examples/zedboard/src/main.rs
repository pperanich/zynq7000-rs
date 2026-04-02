#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use log::{error, info};
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_embassy::{Config as EmbassyConfig, bind_interrupts};
use zynq7000_hal::{BootMode, gpio, uart};

use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard GPIO blinky example --\n\r";

bind_interrupts!(struct Irqs {
    GlobalTimer => zynq7000_embassy::time::InterruptHandler;
});

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let platform = zynq7000_embassy::init(EmbassyConfig {
        ps_clock_frequency: PS_CLOCK_FREQUENCY,
        hal: zynq7000_hal::Config {
            init_l2_cache: true,
            level_shifter_config: Some(zynq7000_hal::LevelShifterConfig::EnableAll),
            interrupt_config: Some(zynq7000_hal::InterruptConfig::AllInterruptsToCpu0),
        },
    })
    .unwrap();
    let periphs = platform.peripherals;
    let clocks = platform.clocks;

    let mut gpio_pins = gpio::GpioPins::new(periphs.gpio);

    // Set up the UART, we are logging with it.
    let uart_clk_config = uart::ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut uart = uart::TypedUart::<uart::Uart1>::new_with_mio(
        periphs.uart_1,
        uart::Config::new_with_clk_config(uart_clk_config),
        (gpio_pins.mio.mio48, gpio_pins.mio.mio49),
    )
    .unwrap();
    uart.write_all(INIT_STRING.as_bytes()).unwrap();
    // Safety: We are not multi-threaded yet.
    unsafe {
        zynq7000_hal::log::uart_blocking::init_unsafe_single_core(
            uart,
            log::LevelFilter::Trace,
            false,
        )
    };

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    let mut ticker = Ticker::every(Duration::from_millis(200));

    let mut mio_led = gpio::Output::new_for_mio(gpio_pins.mio.mio7, gpio::PinState::Low);
    let mut emio_leds: [gpio::Output; 8] = [
        gpio::Output::new_for_emio(gpio_pins.emio.take(0).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(1).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(2).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(3).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(4).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(5).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(6).unwrap(), gpio::PinState::Low),
        gpio::Output::new_for_emio(gpio_pins.emio.take(7).unwrap(), gpio::PinState::Low),
    ];
    loop {
        mio_led.toggle().unwrap();

        // Create a wave pattern for emio_leds
        for led in emio_leds.iter_mut() {
            led.toggle().unwrap();
            ticker.next().await; // Wait for the next ticker for each toggle
        }

        ticker.next().await; // Wait for the next cycle of the ticker
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
