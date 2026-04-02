//! CPU private watchdog support built on the Cortex-A9 MPCore watchdog.

use zynq7000::mpcore::{MmioMpCore, WatchdogControl, WatchdogInterruptStatus, WatchdogResetStatus};

const WATCHDOG_DISABLE_STEP_0: u32 = 0x1234_5678;
const WATCHDOG_DISABLE_STEP_1: u32 = 0x8765_4321;

/// Raw watchdog load value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WatchdogTimeout(u32);

impl WatchdogTimeout {
    /// Create a timeout from raw watchdog ticks.
    pub const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }

    /// Return the raw watchdog tick count.
    pub const fn raw(self) -> u32 {
        self.0
    }
}

/// Hardware mode used by the private watchdog.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WatchdogMode {
    /// Timer mode asserts only the PPI interrupt.
    #[default]
    Timer,
    /// Watchdog mode asserts the reset request output on expiry.
    Watchdog,
}

/// Configuration for starting the private watchdog.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WatchdogConfig {
    /// Load value written into the watchdog counter.
    pub timeout: WatchdogTimeout,
    /// 8-bit prescaler applied to the watchdog clock.
    pub prescaler: u8,
    /// Select timer or watchdog mode.
    pub mode: WatchdogMode,
    /// Enable the PPI interrupt path.
    pub irq_enable: bool,
    /// Reload automatically when the counter reaches zero in timer mode.
    pub auto_reload: bool,
}

impl WatchdogConfig {
    /// Construct a watchdog configuration from a timeout value.
    pub const fn new(timeout: WatchdogTimeout) -> Self {
        Self {
            timeout,
            prescaler: 0,
            mode: WatchdogMode::Timer,
            irq_enable: false,
            auto_reload: false,
        }
    }
}

/// CPU private watchdog driver.
pub struct Watchdog {
    regs: MmioMpCore<'static>,
    last_timeout: WatchdogTimeout,
}

impl Watchdog {
    /// Create a driver from the fixed MPCore register block.
    ///
    /// # Safety
    ///
    /// The returned driver aliases the global MPCore register block.
    pub unsafe fn steal() -> Self {
        Self::new(unsafe { zynq7000::mpcore::MpCore::new_mmio_fixed() })
    }

    /// Create a driver from an MPCore MMIO handle.
    pub const fn new(regs: MmioMpCore<'static>) -> Self {
        Self {
            regs,
            last_timeout: WatchdogTimeout::from_raw(0),
        }
    }

    /// Start or reconfigure the watchdog.
    pub fn start(&mut self, config: WatchdogConfig) {
        self.last_timeout = config.timeout;
        self.regs.write_watchdog_load(config.timeout.raw());
        self.regs.write_watchdog_ctrl(
            WatchdogControl::builder()
                .with_prescaler(config.prescaler)
                .with_watchdog_mode(matches!(config.mode, WatchdogMode::Watchdog))
                .with_it_enable(config.irq_enable)
                .with_auto_reload(config.auto_reload)
                .with_watchdog_enable(true)
                .build(),
        );
    }

    /// Feed the watchdog by reloading the last configured timeout value.
    pub fn feed(&mut self) {
        self.regs.write_watchdog_load(self.last_timeout.raw());
    }

    /// Stop the watchdog and leave it in timer mode.
    pub fn disable(&mut self) {
        self.regs.write_watchdog_disable(WATCHDOG_DISABLE_STEP_0);
        self.regs.write_watchdog_disable(WATCHDOG_DISABLE_STEP_1);
        self.regs.modify_watchdog_ctrl(|mut val| {
            val.set_watchdog_enable(false);
            val.set_it_enable(false);
            val.set_auto_reload(false);
            val
        });
    }

    /// Read the current watchdog counter value.
    pub fn counter(&self) -> u32 {
        self.regs.read_watchdog_counter()
    }

    /// Return the last configured timeout value.
    pub const fn last_timeout(&self) -> WatchdogTimeout {
        self.last_timeout
    }

    /// Clear the sticky timer-mode interrupt flag.
    pub fn clear_interrupt_flag(&mut self) {
        self.regs
            .write_watchdog_interrupt_status(WatchdogInterruptStatus::ack_event_flag());
    }

    /// Read whether timer mode expired since the flag was last cleared.
    pub fn interrupt_flag(&self) -> bool {
        self.regs.read_watchdog_interrupt_status().event_flag()
    }

    /// Clear the sticky watchdog-reset flag.
    pub fn clear_reset_flag(&mut self) {
        self.regs
            .write_watchdog_reset_status(WatchdogResetStatus::ack_reset_flag());
    }

    /// Read whether watchdog mode has already asserted the reset request output.
    pub fn reset_flag(&self) -> bool {
        self.regs.read_watchdog_reset_status().reset_flag()
    }
}
