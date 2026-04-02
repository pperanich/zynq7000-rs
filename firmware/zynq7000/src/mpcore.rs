//! Application Processing Unit Registers (mpcore)
//!
//! Based on p.1483 of the Zynq-7000 TRM.
use static_assertions::const_assert_eq;

use crate::{
    gic::{
        CpuInterfaceRegisters, DistributorRegisters, MmioCpuInterfaceRegisters,
        MmioDistributorRegisters,
    },
    gtc::{MmioRegisters, Registers},
};

pub const MPCORE_BASE_ADDR: usize = 0xF8F0_0000;
pub const SCU_BASE_ADDR: usize = MPCORE_BASE_ADDR;
pub const GICC_BASE_ADDR: usize = MPCORE_BASE_ADDR + 0x100;
pub const GICD_BASE_ADDR: usize = MPCORE_BASE_ADDR + 0x1000;

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct PrivateTimerControl {
    #[bits(8..=15, rw)]
    prescaler: u8,
    #[bit(2, rw)]
    irq_enable: bool,
    #[bit(1, rw)]
    auto_reload: bool,
    #[bit(0, rw)]
    timer_enable: bool,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct PrivateTimerInterruptStatus {
    #[bit(0, rw)]
    event_flag: bool,
}

impl PrivateTimerInterruptStatus {
    /// Builds a zero-based W1C write that acknowledges the private timer event flag.
    pub const fn ack_event_flag() -> Self {
        Self::new_with_raw_value(1)
    }
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct WatchdogControl {
    #[bits(8..=15, rw)]
    prescaler: u8,
    #[bit(3, rw)]
    watchdog_mode: bool,
    #[bit(2, rw)]
    it_enable: bool,
    #[bit(1, rw)]
    auto_reload: bool,
    #[bit(0, rw)]
    watchdog_enable: bool,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct WatchdogInterruptStatus {
    #[bit(0, rw)]
    event_flag: bool,
}

impl WatchdogInterruptStatus {
    /// Builds a zero-based W1C write that acknowledges the watchdog event flag.
    pub const fn ack_event_flag() -> Self {
        Self::new_with_raw_value(1)
    }
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct WatchdogResetStatus {
    #[bit(0, rw)]
    reset_flag: bool,
}

impl WatchdogResetStatus {
    /// Builds a zero-based W1C write that acknowledges the watchdog reset flag.
    pub const fn ack_reset_flag() -> Self {
        Self::new_with_raw_value(1)
    }
}

#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct SnoopControlUnit {
    control: u32,
    config: u32,
    cpu_power_status: u32,
    invalidate_all_regs_in_secure_state: u32,
    _reserved_0: [u32; 0xC],
    filtering_start_addr: u32,
    filtering_end_addr: u32,
    _reserved_1: [u32; 0x2],
    access_ctrl: u32,
    non_secure_access_ctrl: u32,
}

impl SnoopControlUnit {
    /// Create a new Snoop Control Unit interface at the fixed base address.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioSnoopControlUnit<'static> {
        unsafe { Self::new_mmio_at(SCU_BASE_ADDR) }
    }
}

const_assert_eq!(core::mem::size_of::<SnoopControlUnit>(), 0x58);

/// MP Core register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct MpCore {
    #[mmio(Inner)]
    scu: SnoopControlUnit,

    _reserved_0: [u32; 0x2A],

    #[mmio(Inner)]
    gicc: CpuInterfaceRegisters,

    #[mmio(Inner)]
    gt: Registers,

    _reserved_1: [u32; 0xF9],

    private_timer_load: u32,
    private_timer_counter: u32,
    private_timer_ctrl: PrivateTimerControl,
    private_interrupt_status: PrivateTimerInterruptStatus,

    _reserved_2: [u32; 0x4],

    watchdog_load: u32,
    watchdog_counter: u32,
    watchdog_ctrl: WatchdogControl,
    watchdog_interrupt_status: WatchdogInterruptStatus,
    watchdog_reset_status: WatchdogResetStatus,
    watchdog_disable: u32,

    _reserved_3: [u32; 0x272],

    #[mmio(Inner)]
    gicd: DistributorRegisters,
}

const_assert_eq!(core::mem::size_of::<MpCore>(), 0x2000);

impl MpCore {
    /// Create a MP core peripheral interface at the fixed base address.
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    #[inline]
    pub const unsafe fn new_mmio_fixed() -> MmioMpCore<'static> {
        unsafe { Self::new_mmio_at(MPCORE_BASE_ADDR) }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn private_timer_ack_event_flag_sets_only_bit_zero() {
        assert_eq!(PrivateTimerInterruptStatus::ack_event_flag().raw_value(), 1);
    }

    #[test]
    fn watchdog_ack_event_flag_sets_only_bit_zero() {
        assert_eq!(WatchdogInterruptStatus::ack_event_flag().raw_value(), 1);
    }

    #[test]
    fn watchdog_ack_reset_flag_sets_only_bit_zero() {
        assert_eq!(WatchdogResetStatus::ack_reset_flag().raw_value(), 1);
    }
}
