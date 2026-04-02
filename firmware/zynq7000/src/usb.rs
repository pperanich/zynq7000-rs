//! PS USB controller register module.
//!
//! The Zynq-7000 PS USB controller is based on the ChipIdea device/host core. This module only
//! models the subset of registers needed for the current device-mode HAL bring-up work.

use arbitrary_int::{u5, u6, u7, u21};

pub const USB_0_BASE_ADDR: usize = 0xE000_2000;
pub const USB_1_BASE_ADDR: usize = 0xE000_3000;

const USB_STATUS_ACK_MASK: u32 =
    (1 << 8) | (1 << 7) | (1 << 6) | (1 << 4) | (1 << 2) | (1 << 1) | 1;
const ENDPOINT_SETUP_STATUS_ACK_MASK: u32 = 0xFFFF;
const PORTSC1_CHANGE_ACK_MASK: u32 = (1 << 5) | (1 << 3) | (1 << 1);
const OTGSC_POWER_DETECTION_INTERRUPT_STATUS_MASK: u32 =
    (1 << 20) | (1 << 19) | (1 << 18) | (1 << 17);
const OTGSC_POWER_DETECTION_INTERRUPT_ENABLE_MASK: u32 =
    (1 << 28) | (1 << 27) | (1 << 26) | (1 << 25);

/// USB controller mode.
#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq, Eq)]
pub enum ControllerMode {
    Idle = 0b00,
    Reserved = 0b01,
    Device = 0b10,
    Host = 0b11,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct UsbCommand {
    #[bit(14, rw)]
    add_dtd_tripwire: bool,
    #[bit(13, rw)]
    setup_tripwire: bool,
    #[bit(1, rw)]
    controller_reset: bool,
    #[bit(0, rw)]
    run_stop: bool,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct UsbStatus {
    #[bit(8, rw)]
    suspend: bool,
    #[bit(7, rw)]
    sof_received: bool,
    #[bit(6, rw)]
    reset_received: bool,
    #[bit(4, rw)]
    system_error: bool,
    #[bit(2, rw)]
    port_change_detect: bool,
    #[bit(1, rw)]
    usb_error_interrupt: bool,
    #[bit(0, rw)]
    usb_interrupt: bool,
}

impl UsbStatus {
    /// Builds a zero-based W1C write that acknowledges the pending USB status bits in `status`.
    pub const fn ack_from(status: Self) -> Self {
        Self::new_with_raw_value(status.raw_value() & USB_STATUS_ACK_MASK)
    }
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct UsbInterrupt {
    #[bit(8, rw)]
    suspend: bool,
    #[bit(7, rw)]
    sof_received: bool,
    #[bit(6, rw)]
    reset_received: bool,
    #[bit(4, rw)]
    system_error: bool,
    #[bit(2, rw)]
    port_change_detect: bool,
    #[bit(1, rw)]
    usb_error_interrupt: bool,
    #[bit(0, rw)]
    usb_interrupt: bool,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct DeviceAddress {
    #[bits(25..=31, rw)]
    usb_address: u7,
    #[bit(24, rw)]
    address_advance: bool,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct EndpointListAddress {
    #[bits(11..=31, rw)]
    base_address: u21,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct BurstSize {
    #[bits(8..=15, rw)]
    tx_burst: u8,
    #[bits(0..=7, rw)]
    rx_burst: u8,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct TxFillTuning {
    #[bits(16..=20, rw)]
    tx_fifo_threshold: u5,
    #[bits(8..=12, rw)]
    tx_schoh: u5,
    #[bits(0..=5, rw)]
    tx_sc: u6,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct EndpointBitmap {
    #[bits(16..=31, rw)]
    tx: u16,
    #[bits(0..=15, rw)]
    rx: u16,
}

impl EndpointBitmap {
    /// Builds a zero-based W1C write that acknowledges the pending endpoint bitmap bits in
    /// `status`.
    pub const fn ack_from(status: Self) -> Self {
        Self::new_with_raw_value(status.raw_value())
    }
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct PortSc1 {
    #[bit(1, rw)]
    connect_status_change: bool,
    #[bit(3, rw)]
    port_enable_disable_change: bool,
    #[bit(5, rw)]
    overcurrent_change: bool,
    #[bit(8, rw)]
    port_reset: bool,
    #[bit(7, rw)]
    suspend: bool,
    #[bit(0, r)]
    current_connect_status: bool,
}

impl PortSc1 {
    /// Builds a zero-based W1C write that acknowledges the selected port change bits.
    pub const fn ack_change_bits(
        connect_status_change: bool,
        port_enable_disable_change: bool,
        overcurrent_change: bool,
    ) -> Self {
        let mut raw = 0;
        if connect_status_change {
            raw |= 1 << 1;
        }
        if port_enable_disable_change {
            raw |= 1 << 3;
        }
        if overcurrent_change {
            raw |= 1 << 5;
        }
        Self::new_with_raw_value(raw)
    }

    /// Builds a mixed register write that preserves the current register image while
    /// acknowledging the pending port change bits in `port`.
    pub const fn ack_changes_from(port: Self) -> Self {
        let mut raw = port.raw_value();
        raw |= port.raw_value() & PORTSC1_CHANGE_ACK_MASK;
        Self::new_with_raw_value(raw)
    }
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct OtgSc {
    #[bit(28, rw)]
    b_session_end_interrupt_enable: bool,
    #[bit(27, rw)]
    b_session_valid_interrupt_enable: bool,
    #[bit(26, rw)]
    a_session_valid_interrupt_enable: bool,
    #[bit(25, rw)]
    a_vbus_valid_interrupt_enable: bool,
    #[bit(20, rw)]
    b_session_end_interrupt_status: bool,
    #[bit(19, rw)]
    b_session_valid_interrupt_status: bool,
    #[bit(18, rw)]
    a_session_valid_interrupt_status: bool,
    #[bit(17, rw)]
    a_vbus_valid_interrupt_status: bool,
    #[bit(12, r)]
    b_session_end: bool,
    #[bit(11, r)]
    b_session_valid: bool,
    #[bit(10, r)]
    a_session_valid: bool,
    #[bit(9, r)]
    a_vbus_valid: bool,
}

impl OtgSc {
    /// Builds a mixed register write that preserves the current register image while updating
    /// the power-detection interrupt enables and acknowledging any latched power-detection
    /// interrupt status bits in `clear_from`.
    pub const fn power_detection_irq_write(enabled: bool, clear_from: Self) -> Self {
        let mut raw = clear_from.raw_value();
        raw &= !OTGSC_POWER_DETECTION_INTERRUPT_ENABLE_MASK;
        if enabled {
            raw |= OTGSC_POWER_DETECTION_INTERRUPT_ENABLE_MASK;
        }
        Self::new_with_raw_value(raw)
    }
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct UsbMode {
    #[bit(4, rw)]
    stream_disable_mode: bool,
    #[bit(3, rw)]
    setup_lockout_mode: bool,
    #[bits(0..=1, rw)]
    controller_mode: ControllerMode,
}

#[bitbybit::bitfield(u32, default = 0x0, debug)]
pub struct EndpointSetupStatus {
    #[bits(0..=15, rw)]
    setup_endpoints: u16,
}

impl EndpointSetupStatus {
    /// Builds a zero-based W1C write that acknowledges the selected setup endpoint bits.
    pub const fn ack_mask(mask: u16) -> Self {
        Self::new_with_raw_value(mask as u32)
    }

    /// Builds a zero-based W1C write that acknowledges the pending setup endpoint bits in
    /// `status`.
    pub const fn ack_from(status: Self) -> Self {
        Self::new_with_raw_value(status.raw_value() & ENDPOINT_SETUP_STATUS_ACK_MASK)
    }
}

/// USB register access.
#[derive(derive_mmio::Mmio)]
#[repr(C)]
pub struct Registers {
    #[mmio(PureRead)]
    id: u32,
    #[mmio(PureRead)]
    hwgeneral: u32,
    #[mmio(PureRead)]
    hwhost: u32,
    #[mmio(PureRead)]
    hwdevice: u32,
    #[mmio(PureRead)]
    hwtxbuf: u32,
    #[mmio(PureRead)]
    hwrxbuf: u32,
    _gap0: [u32; 74],
    usbcmd: UsbCommand,
    #[mmio(PureRead, Write)]
    usbsts: UsbStatus,
    usbintr: UsbInterrupt,
    #[mmio(PureRead)]
    frindex: u32,
    _gap1: u32,
    deviceaddr: DeviceAddress,
    endpointlistaddr: EndpointListAddress,
    _gap2: u32,
    burstsize: BurstSize,
    txfilltuning: TxFillTuning,
    _gap3: [u32; 4],
    #[mmio(PureRead)]
    endptnak: EndpointBitmap,
    endptnaken: EndpointBitmap,
    configflag: u32,
    portsc1: PortSc1,
    _gap4: [u32; 7],
    otgsc: OtgSc,
    usbmode: UsbMode,
    #[mmio(PureRead, Write)]
    endptsetupstat: EndpointSetupStatus,
    endptprime: EndpointBitmap,
    endptflush: EndpointBitmap,
    #[mmio(PureRead)]
    endptstatus: EndpointBitmap,
    #[mmio(PureRead, Write)]
    endptcomplete: EndpointBitmap,
    endptctrl: [u32; 8],
}

impl Registers {
    /// Create a new USB MMIO instance for USB 0 at address [USB_0_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_0() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(USB_0_BASE_ADDR) }
    }

    /// Create a new USB MMIO instance for USB 1 at address [USB_1_BASE_ADDR].
    ///
    /// # Safety
    ///
    /// This API can be used to potentially create a driver to the same peripheral structure
    /// from multiple threads. The user must ensure that concurrent accesses are safe and do not
    /// interfere with each other.
    pub const unsafe fn new_mmio_fixed_1() -> MmioRegisters<'static> {
        unsafe { Self::new_mmio_at(USB_1_BASE_ADDR) }
    }
}

static_assertions::const_assert_eq!(core::mem::size_of::<Registers>(), 0x1E0);

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn usb_status_ack_from_keeps_only_w1c_status_bits() {
        let status = UsbStatus::new_with_raw_value(0xFFFF_FFFF);
        assert_eq!(UsbStatus::ack_from(status).raw_value(), USB_STATUS_ACK_MASK);
    }

    #[test]
    fn endpoint_setup_status_ack_helpers_build_zero_based_masks() {
        assert_eq!(EndpointSetupStatus::ack_mask(0x55AA).raw_value(), 0x55AA);
        assert_eq!(
            EndpointSetupStatus::ack_from(EndpointSetupStatus::new_with_raw_value(0xABCD_55AA))
                .raw_value(),
            0x55AA
        );
    }

    #[test]
    fn endpoint_bitmap_ack_from_replays_only_bitmap_bits() {
        let status = EndpointBitmap::new_with_raw_value(0xA5A5_5A5A);
        assert_eq!(EndpointBitmap::ack_from(status).raw_value(), 0xA5A5_5A5A);
    }

    #[test]
    fn portsc1_ack_change_bits_sets_only_change_flags() {
        let ack = PortSc1::ack_change_bits(true, false, true);
        assert_eq!(ack.raw_value(), (1 << 5) | (1 << 1));
    }

    #[test]
    fn portsc1_ack_changes_from_preserves_control_bits() {
        let port = PortSc1::new_with_raw_value(PORTSC1_CHANGE_ACK_MASK | (1 << 8) | (1 << 7) | 1);
        assert_eq!(
            PortSc1::ack_changes_from(port).raw_value(),
            PORTSC1_CHANGE_ACK_MASK | (1 << 8) | (1 << 7) | 1
        );
    }

    #[test]
    fn otgsc_power_detection_irq_write_preserves_other_bits_and_updates_enables() {
        let source = OtgSc::new_with_raw_value(
            OTGSC_POWER_DETECTION_INTERRUPT_STATUS_MASK
                | (1 << 12)
                | (1 << 11)
                | (1 << 10)
                | (1 << 9),
        );
        assert_eq!(
            OtgSc::power_detection_irq_write(true, source).raw_value(),
            OTGSC_POWER_DETECTION_INTERRUPT_ENABLE_MASK
                | OTGSC_POWER_DETECTION_INTERRUPT_STATUS_MASK
                | (1 << 12)
                | (1 << 11)
                | (1 << 10)
                | (1 << 9)
        );
        assert_eq!(
            OtgSc::power_detection_irq_write(false, source).raw_value(),
            OTGSC_POWER_DETECTION_INTERRUPT_STATUS_MASK
                | (1 << 12)
                | (1 << 11)
                | (1 << 10)
                | (1 << 9)
        );
    }
}
