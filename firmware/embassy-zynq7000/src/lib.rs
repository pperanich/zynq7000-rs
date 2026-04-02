//! Embassy-first HAL support for the AMD Zynq-7000 SoC family.
//!
//! [`zynq7000`] provides the raw PAC and fixed-address register bindings.
//! This crate is the Embassy-oriented HAL layer on top of that PAC:
//! it owns the token inventory, the shared GIC interrupt adaptation,
//! and the peripheral drivers used by Embassy applications.
//!
//! The top-level entrypoint is [`init`], which:
//! - claims the singleton [`Peripherals`] inventory
//! - reads the current clock tree and acquires the PAC singleton
//! - applies level-shifter policy
//! - initializes the shared interrupt runtime
//! - freezes the clocks and starts the Embassy time driver
//! - enables CPU interrupts last
//! - registers the calling core for crate-managed multicore coordination
//! - returns the singleton [`Peripherals`] inventory
//!
//! `init` registers the calling core for crate-managed multicore coordination.
//! Additional cores that will invoke drivers touching shared global state may call
//! [`multicore::register_current_core`] once after their interrupt/runtime setup is complete.
//! This only enables the crate's shared locking and SGI wiring today; the Embassy time driver is
//! still owned by the init core, so secondary-core Embassy executors are not currently supported.
//!
//! Public modules are organized by driver domain:
//! [`clocks`] for the frozen clock tree,
//! [`gpio`], [`i2c`], [`qspi`], [`uart`], [`usb`], and [`ttc`] for peripheral drivers,
//! and [`interrupt`] / [`time`] for Embassy runtime integration.
//!
//! Interrupt bindings are intentionally Zynq-specific: one crate-owned physical IRQ entrypoint
//! fans into logical Embassy bindings through [`interrupt::bind`] and [`bind_interrupts!`].
//!
//! Feature flags:
//! - `defmt`: enable `defmt` formatting support throughout the crate.
//! - `7z010-7z007s-clg225`: restrict package-sensitive pin inventories to the 32-MIO CLG225
//!   devices (`7z010` / `7z007s`).
#![no_std]

mod cache;
pub mod clocks;
pub mod gpio;
pub mod i2c;
pub mod interrupt;
pub mod log;
pub mod multicore;
pub mod qspi;
pub mod sdmmc;
pub mod time;
pub mod ttc;
pub mod uart;
pub mod usb;

mod chip;
mod gtc;
mod init;
mod l2_cache;
mod runtime;
mod slcr;

pub use embassy_hal_internal::{Peri, PeripheralType};
pub use zynq7000 as pac;
pub use zynq7000::slcr::LevelShifterConfig;
pub type Hertz = fugit::HertzU32;

pub use crate::chip::{Peripherals, peripherals};

/// High-level Embassy interrupt initialization policy.
#[derive(Debug, Clone, Copy)]
pub enum InterruptConfig {
    /// Configure the GIC to route all interrupts to CPU0.
    AllInterruptsToCpu0,
}

/// L2 cache/DMA platform policy used during initialization.
#[derive(Debug, Clone, Copy)]
pub enum L2CacheMode {
    /// Initialize the PL310/L2C block with the crate defaults.
    Initialize,
    /// Assume the boot chain has already initialized L2C for DMA-safe cache maintenance.
    AssumeInitializedForDma,
}

/// High-level Embassy platform initialization config.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub ps_clock_frequency: Hertz,
    pub l2_cache_mode: L2CacheMode,
    pub level_shifter_config: Option<LevelShifterConfig>,
    pub interrupt_config: Option<InterruptConfig>,
}

/// Initialization errors for the top-level Embassy init path.
#[derive(Debug, thiserror::Error)]
pub enum InitError {
    #[error("peripheral singleton was already taken")]
    PeripheralsAlreadyTaken,
    #[error(transparent)]
    Clocks(#[from] clocks::ClockReadError),
}

/// Perform bring-up, freeze the clocks, initialize the Embassy time driver, enable CPU
/// interrupts, and return the token inventory.
///
/// The singleton [`Peripherals`] inventory is claimed before any hardware state is mutated, so a
/// repeated call fails before partially reconfiguring the platform.
pub fn init(config: Config) -> Result<Peripherals, InitError> {
    init::init(config)
}

#[::zynq7000_rt::irq]
fn irq_handler() {
    let _ = interrupt::dispatch_current();
}

/// Bind logical Zynq interrupt sources to Embassy-aware handlers.
#[macro_export]
macro_rules! bind_interrupts {
    ($(#[$attr:meta])* $vis:vis struct $name:ident {
        $(
            $(#[cfg($cond_irq:meta)])?
            $irq:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        #[derive(Copy, Clone)]
        $(#[$attr])*
        $vis struct $name;

        $(
            $(#[cfg($cond_irq)])?
            $crate::__bind_interrupts_impls!($name, $irq, $(
                $(#[cfg($cond_handler)])?
                $handler
            ),*);
        )*
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __bind_interrupts_impls {
    ($name:ident, $irq:ident, $( $(#[cfg($cond_handler:meta)])? $handler:ty ),* $(,)?) => {
        $crate::__bind_interrupts_impls!(@inner $name, $irq, $( $(#[cfg($cond_handler)])? $handler ),*);
    };
    (@inner $name:ident, $irq:ident, $( $(#[cfg($cond_handler:meta)])? $handler:ty ),* $(,)?) => {
        $(
            $(#[cfg($cond_handler)])?
            $crate::__bind_interrupts_impls!(@single $name, $irq, $handler);
        )*
    };
    (@single $name:ident, $irq:ident, $handler:ty) => {
        unsafe impl $crate::interrupt::typelevel::Binding<
            $crate::interrupt::typelevel::$irq,
            $handler
        > for $name {
            fn register() {
                $crate::interrupt::register::<
                    $crate::interrupt::typelevel::$irq,
                    $handler
                >();
            }
        }
    };
}
