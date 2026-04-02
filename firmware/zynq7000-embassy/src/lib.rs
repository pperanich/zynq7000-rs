//! Embassy integration support for the AMD Zynq7000 SoC family.
#![no_std]

pub mod time;

use zynq7000_hal::{System, SystemConfig, SystemInitError, gtc::GlobalTimerCounter};

/// High-level Embassy platform initialization config.
pub type Config = SystemConfig;

/// Initialized Zynq platform state for Embassy applications.
pub type Platform = System;

/// Initialization errors for the high-level Embassy init path.
pub type InitError = SystemInitError;

/// Perform HAL bring-up, clock calculation and Embassy time setup.
pub fn init(config: Config) -> Result<Platform, InitError> {
    let system = zynq7000_hal::init_system(config)?;
    let gtc =
        unsafe { GlobalTimerCounter::steal_fixed(Some(system.clocks.arm_clocks().cpu_3x2x_clk())) };
    time::init(system.clocks.arm_clocks(), gtc);
    Ok(system)
}

/// Dispatch one pending GIC interrupt using a driver-provided matcher.
pub fn dispatch_interrupts(mut dispatch: impl FnMut(zynq7000_hal::gic::Interrupt) -> bool) -> bool {
    let mut gic_helper = zynq7000_hal::gic::GicInterruptHelper::new();
    let irq_info = gic_helper.acknowledge_interrupt();
    let handled = match irq_info.interrupt() {
        zynq7000_hal::gic::Interrupt::Spurious
        | zynq7000_hal::gic::Interrupt::Invalid(_)
        | zynq7000_hal::gic::Interrupt::Sgi(_) => false,
        interrupt => dispatch(interrupt),
    };
    gic_helper.end_of_interrupt(irq_info);
    handled
}

/// Bind logical Zynq interrupt sources to Embassy-aware handlers.
#[macro_export]
macro_rules! bind_interrupts {
    ($(#[$attr:meta])* $vis:vis struct $name:ident {
        $(
            $irq:ident => $handler:ty;
        )*
    }) => {
        #[derive(Copy, Clone)]
        $(#[$attr])*
        $vis struct $name;

        $(
            unsafe impl ::zynq7000_hal::interrupt::typelevel::Binding<
                ::zynq7000_hal::interrupt::typelevel::$irq,
                $handler
            > for $name {}
        )*

        #[::zynq7000_rt::irq]
        fn irq_handler() {
            let _ = ::zynq7000_embassy::dispatch_interrupts(|interrupt| match interrupt {
                $(
                    irq if irq == <::zynq7000_hal::interrupt::typelevel::$irq as ::zynq7000_hal::interrupt::typelevel::Interrupt>::IRQ => {
                        unsafe {
                            <$handler as ::zynq7000_hal::interrupt::typelevel::Handler<
                                ::zynq7000_hal::interrupt::typelevel::$irq
                            >>::on_interrupt();
                        }
                        true
                    }
                )*
                _ => false,
            });
        }
    };
}
