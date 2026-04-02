#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use axi_uart16550::AxiUart16550;
use axi_uartlite::AxiUartlite;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io::Write;
use fugit::RateExtU32;
use log::{error, info};
use zedboard::PS_CLOCK_FREQUENCY;
use zynq7000_hal::{
    BootMode,
    gpio::{GpioPins, Output, PinState},
    gtc::GlobalTimerCounter,
    uart::{ClockConfig, Config, Uart},
};

use zynq7000::slcr::LevelShifterConfig;
use zynq7000_rt as _;

const INIT_STRING: &str = "-- Zynq 7000 Zedboard blocking UART example --\n\r";

const AXI_UARTLITE_BASE_ADDR: u32 = 0x42C0_0000;
const AXI_UAR16550_BASE_ADDR: u32 = 0x43C0_0000;

/// Entry point which calls the embassy main method.
#[zynq7000_rt::entry]
fn entry_point() -> ! {
    main();
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum UartSel {
    Uart0 = 0b000,
    Uartlite = 0b001,
    Uart16550 = 0b010,
    Uart0ToUartlite = 0b011,
    Uart0ToUart16550 = 0b100,
    UartliteToUart16550 = 0b101,
}

pub struct UartMultiplexer {
    sel_pins: [Output; 3],
}

impl UartMultiplexer {
    pub fn new(mut sel_pins: [Output; 3]) -> Self {
        for pin in sel_pins.iter_mut() {
            pin.set_low();
        }
        Self { sel_pins }
    }

    pub fn select(&mut self, sel: UartSel) {
        // TODO: A pin group switcher would be nice to do this in one go.
        match sel {
            UartSel::Uart0 => {
                self.sel_pins[2].set_low();
                self.sel_pins[1].set_low();
                self.sel_pins[0].set_low();
            }
            UartSel::Uartlite => {
                self.sel_pins[2].set_low();
                self.sel_pins[1].set_low();
                self.sel_pins[0].set_high();
            }
            UartSel::Uart16550 => {
                self.sel_pins[2].set_low();
                self.sel_pins[1].set_high();
                self.sel_pins[0].set_low();
            }
            UartSel::Uart0ToUartlite => {
                self.sel_pins[2].set_low();
                self.sel_pins[1].set_high();
                self.sel_pins[0].set_high();
            }
            UartSel::Uart0ToUart16550 => {
                self.sel_pins[2].set_high();
                self.sel_pins[1].set_low();
                self.sel_pins[0].set_low();
            }
            UartSel::UartliteToUart16550 => {
                self.sel_pins[2].set_high();
                self.sel_pins[1].set_low();
                self.sel_pins[0].set_high();
            }
        }
    }
}
#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let system = zynq7000_hal::init_system(zynq7000_hal::SystemConfig {
        ps_clock_frequency: PS_CLOCK_FREQUENCY,
        hal: zynq7000_hal::Config {
            init_l2_cache: true,
            level_shifter_config: Some(LevelShifterConfig::EnableAll),
            interrupt_config: Some(zynq7000_hal::InterruptConfig::AllInterruptsToCpu0),
        },
    })
    .unwrap();
    let (dp, clocks) = system.into_parts();
    let mut gpio_pins = GpioPins::new(dp.gpio);

    // Set up global timer counter and embassy time driver.
    let gtc = GlobalTimerCounter::new(dp.gtc, clocks.arm_clocks());
    zynq7000_embassy::time::init(clocks.arm_clocks(), gtc);

    // Set up the UART, we are logging with it.
    let uart_clk_config = ClockConfig::new_autocalc_with_error(clocks.io_clocks(), 115200)
        .unwrap()
        .0;
    let mut log_uart = zynq7000_hal::uart::TypedUart::<zynq7000_hal::uart::Uart1>::new_with_mio(
        dp.uart_1,
        Config::new_with_clk_config(uart_clk_config),
        (gpio_pins.mio.mio48, gpio_pins.mio.mio49),
    )
    .unwrap();
    log_uart.write_all(INIT_STRING.as_bytes()).unwrap();

    // Safety: Co-operative multi-tasking is used.
    unsafe {
        zynq7000_hal::log::uart_blocking::init_unsafe_single_core(
            log_uart,
            log::LevelFilter::Trace,
            false,
        )
    };

    // UART0 routed through EMIO to PL pins.
    let mut uart_0 = Uart::new_typed_with_emio::<zynq7000_hal::uart::Uart0>(
        dp.uart_0,
        Config::new_with_clk_config(uart_clk_config),
    )
    .unwrap();
    // Safety: Valid address of AXI UARTLITE.
    let mut uartlite = unsafe { AxiUartlite::new(AXI_UARTLITE_BASE_ADDR) };

    // TODO: Can we determine/read the clock frequency to the FPGAs as well?
    let (clk_config, error) =
        axi_uart16550::ClockConfig::new_autocalc_with_error(100.MHz(), 115200).unwrap();
    assert!(error < 0.02);
    let mut uart_16550 = unsafe {
        AxiUart16550::new(
            AXI_UAR16550_BASE_ADDR,
            axi_uart16550::UartConfig::new_with_clk_config(clk_config),
        )
    };

    let boot_mode = BootMode::new_from_regs();
    info!("Boot mode: {:?}", boot_mode);

    let mut ticker = Ticker::every(Duration::from_millis(1000));

    let mut mio_led = Output::new_for_mio(gpio_pins.mio.mio7, PinState::Low);
    let mut emio_leds: [Output; 8] = [
        Output::new_for_emio(gpio_pins.emio.take(0).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(1).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(2).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(3).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(4).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(5).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(6).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(7).unwrap(), PinState::Low),
    ];

    let mut uart_mux = UartMultiplexer::new([
        Output::new_for_emio(gpio_pins.emio.take(8).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(9).unwrap(), PinState::Low),
        Output::new_for_emio(gpio_pins.emio.take(10).unwrap(), PinState::Low),
    ]);
    let mut current_sel = UartSel::Uart0;
    uart_mux.select(current_sel);
    let mut led_idx = 0;
    loop {
        mio_led.toggle().unwrap();

        emio_leds[led_idx].toggle().unwrap();
        led_idx += 1;
        if led_idx >= emio_leds.len() {
            led_idx = 0;
        }
        uart_0
            .write_all("Hello, World from UART0!\n\r".as_bytes())
            .unwrap();
        uartlite
            .write_all("Hello, World from AXI UARTLITE!\n\r".as_bytes())
            .unwrap();
        uart_16550
            .write_all("Hello, World from AXI UART16550!\n\r".as_bytes())
            .unwrap();

        uart_0.flush().unwrap();
        uartlite.flush().unwrap();
        uart_16550.flush().unwrap();
        match current_sel {
            UartSel::Uart0 => current_sel = UartSel::Uartlite,
            UartSel::Uartlite => current_sel = UartSel::Uart16550,
            UartSel::Uart16550 => current_sel = UartSel::Uart0,
            UartSel::Uart0ToUartlite | UartSel::Uart0ToUart16550 | UartSel::UartliteToUart16550 => {
            }
        }
        uart_mux.select(current_sel);
        ticker.next().await; // Wait for the next cycle of the ticker
    }
}

#[zynq7000_rt::irq]
fn irq_handler() {
    let _ = zynq7000_embassy::dispatch_interrupts(|interrupt| match interrupt {
        zynq7000_hal::gic::Interrupt::Ppi(ppi_interrupt)
            if ppi_interrupt == zynq7000_hal::gic::PpiInterrupt::GlobalTimer =>
        {
            unsafe {
                zynq7000_embassy::time::on_interrupt();
            }
            true
        }
        _ => false,
    });
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
