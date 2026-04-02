#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use embassy_executor::Spawner;
use embassy_usb::{Builder, Config};
use embassy_zynq7000::{
    Config as EmbassyConfig, InterruptConfig, L2CacheMode, LevelShifterConfig, bind_interrupts,
};
use embassy_zynq7000::{
    gpio::{self, Output},
    usb as embassy_usb_hal,
};
use log::{error, info, warn};
use static_cell::ConstStaticCell;

use zynq7000_rt as _;

const USB0_RESET_PULSE_CYCLES: usize = 1024;

bind_interrupts!(struct Irqs {
    Usb0 => embassy_zynq7000::usb::InterruptHandler<embassy_zynq7000::peripherals::USB0>;
});

static CONFIG_DESCRIPTOR: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
static BOS_DESCRIPTOR: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
static MSOS_DESCRIPTOR: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
static CONTROL_BUFFER: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);

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
    let mut _usb_reset = init_red_pitaya_usb0_reset(p.MIO48);

    info!("Creating USB0 Embassy device driver");
    warn!("This example wires the stack together but does not yet implement Phase 3 transfers.");

    let driver = embassy_usb_hal::embassy::Driver::new(p.USB0, Irqs);
    let mut config = Config::new(0x1209, 0x0001);
    config.manufacturer = Some("zynq7000-rs");
    config.product = Some("USB skeleton");
    config.serial_number = Some("zynq-usb-stage1");
    config.max_packet_size_0 = 64;
    config.self_powered = true;
    config.max_power = 0;

    let builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.take(),
        BOS_DESCRIPTOR.take(),
        MSOS_DESCRIPTOR.take(),
        CONTROL_BUFFER.take(),
    );
    let mut device = builder.build();

    info!("USB task entering run loop");
    device.run().await;
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

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("Panic: {info:?}");
    loop {}
}

fn init_red_pitaya_usb0_reset(
    pin: embassy_zynq7000::Peri<'static, embassy_zynq7000::peripherals::MIO48>,
) -> Output<'static> {
    let mut reset = Output::new(pin, gpio::PinState::Low);
    for _ in 0..USB0_RESET_PULSE_CYCLES {
        nop();
    }
    reset.set_high();
    reset
}
