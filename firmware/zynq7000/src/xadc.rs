use arbitrary_int::{u2, u4, u5, u7};

pub const XADC_BASE_ADDR: usize = 0xF8007100;
const INTERRUPT_ACK_MASK: u32 = 0x3FF;

#[bitbybit::bitfield(u32, debug)]
pub struct Config {
    #[bit(31, rw)]
    enable: bool,
    #[bits(20..=23, rw)]
    cfifo_threshold: u4,
    #[bits(16..=19, rw)]
    dfifo_threshold: u4,
    #[bit(13, rw)]
    write_data_active_edge: bool,
    #[bit(12, rw)]
    read_data_active_edge: bool,
    #[bits(8..=9, rw)]
    tck_rate: u2,
    #[bits(0..=4, rw)]
    inter_packet_gap: u5,
}

#[bitbybit::bitfield(u32, debug)]
pub struct InterruptStatus {
    #[bit(9, rw)]
    cfifo_below_threshold: bool,
    #[bit(8, rw)]
    dfifo_above_threshold: bool,
    #[bit(7, rw)]
    over_temperature: bool,
    #[bits(0..=6, rw)]
    alarms: u7,
}

impl InterruptStatus {
    /// Builds a zero-based W1C write that acknowledges all pending XADC interrupt bits.
    pub const fn ack_all() -> Self {
        Self::new_with_raw_value(INTERRUPT_ACK_MASK)
    }

    /// Builds a zero-based W1C write that acknowledges the interrupt bits present in `status`.
    pub const fn ack_from(status: Self) -> Self {
        Self::new_with_raw_value(status.raw_value() & INTERRUPT_ACK_MASK)
    }
}

pub type InterruptMask = InterruptStatus;

#[bitbybit::bitfield(u32, debug)]
pub struct MiscStatus {
    #[bits(16..=19, r)]
    cfifo_level: u4,
    #[bits(12..=15, r)]
    dfifo_level: u4,
    #[bit(11, r)]
    cfifo_full: bool,
    #[bit(10, r)]
    cfifo_empty: bool,
    #[bit(9, r)]
    dfifo_full: bool,
    #[bit(8, r)]
    dfifo_empty: bool,
    #[bit(7, r)]
    over_temperature: bool,
    #[bits(0..=6, r)]
    alarms: u7,
}

#[bitbybit::bitfield(u32, debug)]
pub struct CommandFifo {
    #[bits(0..=31, rw)]
    command: u32,
}

#[bitbybit::bitfield(u32, debug)]
pub struct DataFifo {
    #[bits(0..=31, r)]
    read_data: u32,
}

#[bitbybit::bitfield(u32, debug)]
pub struct MiscControl {
    #[bit(4, rw)]
    reset: bool,
}

/// XADC register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    config: Config,
    interrupt_status: InterruptStatus,
    interrupt_mask: InterruptMask,
    misc_status: MiscStatus,
    command_fifo: CommandFifo,
    data_fifo: DataFifo,
    misc_control: MiscControl,
}

impl Registers {
    /// Create a new XADC MMIO instance for for device configuration peripheral at address
    /// [XADC_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub unsafe fn new_mmio_fixed() -> MmioRegisters<'static> {
        unsafe { Registers::new_mmio_at(XADC_BASE_ADDR) }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn ack_all_sets_all_interrupt_bits() {
        assert_eq!(InterruptStatus::ack_all().raw_value(), INTERRUPT_ACK_MASK);
    }

    #[test]
    fn ack_from_masks_out_reserved_bits() {
        let status = InterruptStatus::new_with_raw_value(u32::MAX);
        assert_eq!(
            InterruptStatus::ack_from(status).raw_value(),
            INTERRUPT_ACK_MASK
        );
    }
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x1C);
