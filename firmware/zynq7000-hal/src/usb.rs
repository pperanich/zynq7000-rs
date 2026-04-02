//! USB device-mode HAL support for the Zynq-7000 PS USB controller.
//!
//! Phase 2 of the staged bring-up lives here:
//! - controller reset and device-mode initialization
//! - endpoint queue-head and dTD ownership
//! - cache-safe DMA helpers for descriptors and endpoint buffers
//! - interrupt/state decoding plus a polling smoke-test path for EP0 and one bulk endpoint
//!
//! The current Embassy trait implementation mirrors the low-level transport path closely, but it
//! remains hardware-unproven until board validation confirms the controller behavior.

use core::{
    cell::UnsafeCell,
    future::poll_fn,
    marker::PhantomData,
    mem::MaybeUninit,
    sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering},
    task::Poll,
};

use arbitrary_int::{u7, u15, u21};
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Bus, ControlPipe, Direction, Endpoint, EndpointAddress, EndpointAllocError, EndpointError,
    EndpointIn as EmbassyEndpointIn, EndpointInfo, EndpointOut as EmbassyEndpointOut, EndpointType,
    Event, Unsupported,
};
use zynq7000::usb::{
    ControllerMode, DeviceAddress, EndpointBitmap, EndpointSetupStatus, MmioRegisters, OtgSc,
    PortSc1, UsbInterrupt, UsbMode, UsbStatus,
};

use crate::{PeriphSelect, enable_amba_peripheral_clock, slcr::Slcr};

/// Maximum number of endpoint indices surfaced by the current software allocator.
pub const MAX_ENDPOINTS: usize = 8;
pub const SETUP_PACKET_SIZE: usize = 8;

const QUEUE_HEAD_COUNT: usize = MAX_ENDPOINTS * 2;
const EPCTRL_RX_ENABLE: u32 = 1 << 7;
const EPCTRL_RX_DATA_TOGGLE_RESET: u32 = 1 << 6;
const EPCTRL_RX_TYPE_SHIFT: u32 = 2;
const EPCTRL_RX_STALL: u32 = 1 << 0;
const EPCTRL_TX_ENABLE: u32 = 1 << 23;
const EPCTRL_TX_DATA_TOGGLE_RESET: u32 = 1 << 22;
const EPCTRL_TX_TYPE_SHIFT: u32 = 18;
const EPCTRL_TX_STALL: u32 = 1 << 16;

const DTD_NEXT_TERMINATE: u32 = 1;
const DTD_TOKEN_ACTIVE: u32 = 1 << 7;
const DTD_TOKEN_HALTED: u32 = 1 << 6;
const DTD_TOKEN_BUFFER_ERROR: u32 = 1 << 5;
const DTD_TOKEN_TRANSACTION_ERROR: u32 = 1 << 3;
const DTD_TOKEN_IOC: u32 = 1 << 15;
const DTD_TOTAL_BYTES_SHIFT: u32 = 16;

const QH_CAP_IOS: u32 = 1 << 15;
const QH_CAP_ZLT: u32 = 1 << 29;
const QH_MAX_PACKET_SHIFT: u32 = 16;

const CONTROL_BUFFER_SIZE: usize = 256;
const MAX_DTD_TRANSFER_BYTES: usize = 0x7fff;
const ENDPOINT_FLUSH_TIMEOUT_ITERS: usize = 10_000;

const EVENT_RESET: u32 = 1 << 0;
const EVENT_SUSPEND: u32 = 1 << 1;
const EVENT_RESUME: u32 = 1 << 2;
const EVENT_POWER_DETECTED: u32 = 1 << 3;
const EVENT_POWER_REMOVED: u32 = 1 << 4;
/// USB controller identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsbId {
    /// USB controller 0.
    Usb0,
    /// USB controller 1.
    Usb1,
}

/// Type-level USB0 controller marker for Embassy interrupt bindings.
#[derive(Debug, Clone, Copy)]
pub struct Usb0;

/// Type-level USB1 controller marker for Embassy interrupt bindings.
#[derive(Debug, Clone, Copy)]
pub struct Usb1;

/// Type-level USB controller instance used by Embassy bindings.
pub trait Instance {
    /// Runtime controller identifier.
    const ID: UsbId;
    /// Logical interrupt source bound to this controller.
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

impl Instance for Usb0 {
    const ID: UsbId = UsbId::Usb0;
    type Interrupt = crate::interrupt::typelevel::Usb0;
}

impl Instance for Usb1 {
    const ID: UsbId = UsbId::Usb1;
    type Interrupt = crate::interrupt::typelevel::Usb1;
}

impl UsbId {
    const fn index(self) -> usize {
        match self {
            Self::Usb0 => 0,
            Self::Usb1 => 1,
        }
    }

    const fn periph_select(self) -> PeriphSelect {
        match self {
            Self::Usb0 => PeriphSelect::Usb0,
            Self::Usb1 => PeriphSelect::Usb1,
        }
    }
}

/// Errors returned while bringing the USB controller into device mode.
#[derive(Debug, thiserror::Error)]
pub enum UsbInitError {
    /// The controller did not clear its reset bit within the expected spin-wait window.
    #[error("USB controller reset timed out")]
    ControllerResetTimedOut,
    /// The endpoint queue-head list pointer was not aligned to the controller requirement.
    #[error("queue head list must be 2048-byte aligned")]
    QueueHeadAlignment,
    /// DMA-visible buffers must be cache-line aligned.
    #[error("DMA buffer alignment error")]
    DmaAlignment(#[from] crate::cache::AlignmentError),
}

/// Bus and endpoint events decoded from one interrupt service pass.
#[derive(Debug, Clone, Copy, Default)]
pub struct InterruptResult {
    pub usb_interrupt: bool,
    pub usb_error: bool,
    pub port_change: bool,
    pub reset: bool,
    pub suspend: bool,
    pub resume: bool,
    pub setup_endpoints: u16,
    pub completed_out_endpoints: u16,
    pub completed_in_endpoints: u16,
}

impl InterruptResult {
    pub const fn has_any_event(&self) -> bool {
        self.usb_interrupt
            || self.usb_error
            || self.port_change
            || self.reset
            || self.suspend
            || self.resume
            || self.setup_endpoints != 0
            || self.completed_out_endpoints != 0
            || self.completed_in_endpoints != 0
    }
}

#[repr(C, align(64))]
pub struct EndpointQueueHead {
    capabilities: vcell::VolatileCell<u32>,
    current_dtd: vcell::VolatileCell<u32>,
    next_dtd: vcell::VolatileCell<u32>,
    token: vcell::VolatileCell<u32>,
    buffer_page: [vcell::VolatileCell<u32>; 5],
    reserved: vcell::VolatileCell<u32>,
    setup_data: [vcell::VolatileCell<u32>; 2],
}

impl EndpointQueueHead {
    pub const fn new() -> Self {
        Self {
            capabilities: vcell::VolatileCell::new(0),
            current_dtd: vcell::VolatileCell::new(0),
            next_dtd: vcell::VolatileCell::new(DTD_NEXT_TERMINATE),
            token: vcell::VolatileCell::new(0),
            buffer_page: [const { vcell::VolatileCell::new(0) }; 5],
            reserved: vcell::VolatileCell::new(0),
            setup_data: [const { vcell::VolatileCell::new(0) }; 2],
        }
    }

    pub fn configure(
        &self,
        max_packet_size: u16,
        interrupt_on_setup: bool,
        zero_length_termination: bool,
    ) {
        let mut capabilities = ((max_packet_size as u32) & 0x7ff) << QH_MAX_PACKET_SHIFT;
        if interrupt_on_setup {
            capabilities |= QH_CAP_IOS;
        }
        if zero_length_termination {
            capabilities &= !QH_CAP_ZLT;
        } else {
            capabilities |= QH_CAP_ZLT;
        }
        self.capabilities.set(capabilities);
    }

    pub fn reset_overlay(&self) {
        self.current_dtd.set(0);
        self.next_dtd.set(DTD_NEXT_TERMINATE);
        self.token.set(0);
        for page in &self.buffer_page {
            page.set(0);
        }
    }

    pub fn attach_dtd(&self, dtd: &TransferDescriptor) {
        self.next_dtd
            .set((dtd as *const TransferDescriptor as u32) & !0x1f);
        self.token.set(0);
    }

    pub fn setup_words(&self) -> [u32; 2] {
        [self.setup_data[0].get(), self.setup_data[1].get()]
    }
}

impl Default for EndpointQueueHead {
    fn default() -> Self {
        Self::new()
    }
}

#[repr(C, align(2048))]
pub struct EndpointQueueHeadList {
    qh: [EndpointQueueHead; QUEUE_HEAD_COUNT],
}

impl EndpointQueueHeadList {
    pub const fn new() -> Self {
        Self {
            qh: [const { EndpointQueueHead::new() }; QUEUE_HEAD_COUNT],
        }
    }

    pub fn endpoint(&self, ep_index: usize, dir: Direction) -> &EndpointQueueHead {
        &self.qh[queue_head_index(ep_index, dir)]
    }

    pub fn endpoint_mut(&mut self, ep_index: usize, dir: Direction) -> &mut EndpointQueueHead {
        &mut self.qh[queue_head_index(ep_index, dir)]
    }

    pub fn base_addr(&self) -> u32 {
        self.qh.as_ptr() as u32
    }
}

impl Default for EndpointQueueHeadList {
    fn default() -> Self {
        Self::new()
    }
}

#[repr(transparent)]
pub struct EndpointQueueHeadStorage(pub UnsafeCell<MaybeUninit<EndpointQueueHeadList>>);

unsafe impl Sync for EndpointQueueHeadStorage {}

static QUEUE_HEADS_TAKEN: AtomicBool = AtomicBool::new(false);

impl EndpointQueueHeadStorage {
    pub const fn new() -> Self {
        Self(UnsafeCell::new(MaybeUninit::uninit()))
    }

    pub fn take(&self) -> Option<&'static mut EndpointQueueHeadList> {
        if QUEUE_HEADS_TAKEN.swap(true, Ordering::SeqCst) {
            return None;
        }
        let storage = unsafe { &mut *self.0.get() };
        storage.write(EndpointQueueHeadList::new());
        Some(unsafe { storage.assume_init_mut() })
    }
}

impl Default for EndpointQueueHeadStorage {
    fn default() -> Self {
        Self::new()
    }
}

#[repr(C, align(32))]
pub struct TransferDescriptor {
    next_dtd: vcell::VolatileCell<u32>,
    token: vcell::VolatileCell<u32>,
    buffer_page: [vcell::VolatileCell<u32>; 5],
}

impl TransferDescriptor {
    pub const fn new() -> Self {
        Self {
            next_dtd: vcell::VolatileCell::new(DTD_NEXT_TERMINATE),
            token: vcell::VolatileCell::new(0),
            buffer_page: [const { vcell::VolatileCell::new(0) }; 5],
        }
    }

    pub fn reset(&self) {
        self.next_dtd.set(DTD_NEXT_TERMINATE);
        self.token.set(0);
        for page in &self.buffer_page {
            page.set(0);
        }
    }

    pub fn configure(&self, buf_addr: u32, total_bytes: usize, interrupt_on_complete: bool) {
        self.next_dtd.set(DTD_NEXT_TERMINATE);
        self.buffer_page[0].set(buf_addr);
        for i in 1..self.buffer_page.len() {
            self.buffer_page[i].set((buf_addr & !0xfff).wrapping_add((i as u32) * 0x1000));
        }
        let mut token = ((u15::new(total_bytes as u16).value() as u32) << DTD_TOTAL_BYTES_SHIFT)
            | DTD_TOKEN_ACTIVE;
        if interrupt_on_complete {
            token |= DTD_TOKEN_IOC;
        }
        self.token.set(token);
    }

    pub fn total_bytes(&self) -> usize {
        ((self.token.get() >> DTD_TOTAL_BYTES_SHIFT) & 0x7fff) as usize
    }

    pub fn actual_bytes_transferred(&self, requested: usize) -> usize {
        requested.saturating_sub(self.total_bytes())
    }

    pub fn is_active(&self) -> bool {
        self.token.get() & DTD_TOKEN_ACTIVE != 0
    }

    pub fn has_error(&self) -> bool {
        self.token.get() & (DTD_TOKEN_HALTED | DTD_TOKEN_BUFFER_ERROR | DTD_TOKEN_TRANSACTION_ERROR)
            != 0
    }
}

impl Default for TransferDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

#[repr(C, align(32))]
pub struct AlignedUsbBuffer<const N: usize>(pub [u8; N]);

impl<const N: usize> AlignedUsbBuffer<N> {
    pub const fn new() -> Self {
        Self([0; N])
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.0.as_ptr()
    }

    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.0.as_mut_ptr()
    }
}

impl<const N: usize> Default for AlignedUsbBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Report for a completed USB transfer.
#[derive(Debug, Clone, Copy)]
pub struct TransferReport {
    pub requested_bytes: usize,
    pub actual_bytes: usize,
    pub token: u32,
}

impl TransferReport {
    pub const fn has_error(&self) -> bool {
        self.token & (DTD_TOKEN_HALTED | DTD_TOKEN_BUFFER_ERROR | DTD_TOKEN_TRANSACTION_ERROR) != 0
    }
}

/// Errors returned by the reusable USB device-mode path.
#[derive(Debug, thiserror::Error)]
pub enum UsbError {
    #[error(transparent)]
    Init(#[from] UsbInitError),
    #[error("invalid USB endpoint index {0}")]
    InvalidEndpoint(usize),
    #[error("unsupported USB request")]
    UnsupportedRequest,
    #[error("USB transfer length {0} exceeds dTD capacity")]
    TransferTooLarge(usize),
    #[error("USB transfer requested {requested} bytes from a {buffer_len}-byte buffer")]
    BufferTooSmall { requested: usize, buffer_len: usize },
    #[error("USB endpoint {0:?} is still active")]
    EndpointBusy(EndpointAddress),
    #[error("USB endpoint {0:?} flush timed out")]
    EndpointFlushTimedOut(EndpointAddress),
    #[error("USB transfer on endpoint {addr:?} completed with error token 0x{token:08x}")]
    TransferFailed { addr: EndpointAddress, token: u32 },
}

impl From<crate::cache::AlignmentError> for UsbError {
    fn from(value: crate::cache::AlignmentError) -> Self {
        Self::Init(UsbInitError::DmaAlignment(value))
    }
}

/// Persistent DMA-visible resources for one USB controller instance.
pub struct UsbResources {
    queue_heads: EndpointQueueHeadList,
    dtd_out: [TransferDescriptor; MAX_ENDPOINTS],
    dtd_in: [TransferDescriptor; MAX_ENDPOINTS],
    requested_out: [u16; MAX_ENDPOINTS],
    requested_in: [u16; MAX_ENDPOINTS],
    ep0_out_buffer: AlignedUsbBuffer<CONTROL_BUFFER_SIZE>,
    ep0_in_buffer: AlignedUsbBuffer<CONTROL_BUFFER_SIZE>,
}

impl UsbResources {
    pub const fn new() -> Self {
        Self {
            queue_heads: EndpointQueueHeadList::new(),
            dtd_out: [const { TransferDescriptor::new() }; MAX_ENDPOINTS],
            dtd_in: [const { TransferDescriptor::new() }; MAX_ENDPOINTS],
            requested_out: [0; MAX_ENDPOINTS],
            requested_in: [0; MAX_ENDPOINTS],
            ep0_out_buffer: AlignedUsbBuffer::new(),
            ep0_in_buffer: AlignedUsbBuffer::new(),
        }
    }

    fn reset_state(&mut self) {
        self.queue_heads = EndpointQueueHeadList::new();
        self.dtd_out = [const { TransferDescriptor::new() }; MAX_ENDPOINTS];
        self.dtd_in = [const { TransferDescriptor::new() }; MAX_ENDPOINTS];
        self.requested_out = [0; MAX_ENDPOINTS];
        self.requested_in = [0; MAX_ENDPOINTS];
    }

    fn requested(&self, addr: EndpointAddress) -> usize {
        match addr.direction() {
            Direction::Out => self.requested_out[addr.index()] as usize,
            Direction::In => self.requested_in[addr.index()] as usize,
        }
    }

    fn set_requested(&mut self, addr: EndpointAddress, requested: usize) {
        match addr.direction() {
            Direction::Out => self.requested_out[addr.index()] = requested as u16,
            Direction::In => self.requested_in[addr.index()] = requested as u16,
        }
    }

    fn clear_transfer_state(&mut self, addr: EndpointAddress) {
        self.set_requested(addr, 0);
        self.dtd_mut(addr).reset();
        self.queue_heads
            .endpoint(addr.index(), addr.direction())
            .reset_overlay();
    }

    fn clear_ep0_transfer_state(&mut self) {
        self.clear_transfer_state(ep0_addr(Direction::Out));
        self.clear_transfer_state(ep0_addr(Direction::In));
    }

    fn dtd(&self, addr: EndpointAddress) -> &TransferDescriptor {
        match addr.direction() {
            Direction::Out => &self.dtd_out[addr.index()],
            Direction::In => &self.dtd_in[addr.index()],
        }
    }

    fn dtd_mut(&mut self, addr: EndpointAddress) -> &mut TransferDescriptor {
        match addr.direction() {
            Direction::Out => &mut self.dtd_out[addr.index()],
            Direction::In => &mut self.dtd_in[addr.index()],
        }
    }
}

impl Default for UsbResources {
    fn default() -> Self {
        Self::new()
    }
}

/// Persistent device-mode view which couples one controller with its DMA-visible resources.
pub struct UsbDevice<'a> {
    controller: UsbController,
    resources: &'a mut UsbResources,
}

/// Reusable HAL wrapper for one PS USB controller instance.
pub struct UsbController {
    id: UsbId,
    regs: MmioRegisters<'static>,
}

impl UsbController {
    /// Create a new USB controller wrapper from an owned PAC MMIO handle.
    pub fn new(id: UsbId, regs: MmioRegisters<'static>) -> Self {
        Self { id, regs }
    }

    /// Create a USB0 controller wrapper.
    pub fn usb0(regs: MmioRegisters<'static>) -> Self {
        Self::new(UsbId::Usb0, regs)
    }

    /// Create a USB1 controller wrapper.
    pub fn usb1(regs: MmioRegisters<'static>) -> Self {
        Self::new(UsbId::Usb1, regs)
    }

    /// Controller identifier.
    pub const fn id(&self) -> UsbId {
        self.id
    }

    /// Couple this controller with persistent DMA-visible device resources.
    pub fn into_device<'a>(self, resources: &'a mut UsbResources) -> UsbDevice<'a> {
        UsbDevice {
            controller: self,
            resources,
        }
    }

    /// Put the controller into minimal device mode and enable core interrupts.
    pub fn init_device_mode(&mut self) -> Result<(), UsbInitError> {
        enable_amba_peripheral_clock(self.id.periph_select());
        self.reset_via_slcr();

        self.regs.modify_usbcmd(|mut cmd| {
            cmd.set_run_stop(false);
            cmd.set_controller_reset(true);
            cmd
        });
        for _ in 0..10_000 {
            if !self.regs.read_usbcmd().controller_reset() {
                break;
            }
        }
        if self.regs.read_usbcmd().controller_reset() {
            return Err(UsbInitError::ControllerResetTimedOut);
        }

        let mut mode = UsbMode::new_with_raw_value(0);
        mode.set_controller_mode(ControllerMode::Device);
        mode.set_setup_lockout_mode(false);
        mode.set_stream_disable_mode(true);
        self.regs.write_usbmode(mode);
        self.regs
            .write_usbsts(UsbStatus::ack_from(self.regs.read_usbsts()));
        self.regs
            .write_endptsetupstat(EndpointSetupStatus::ack_from(
                self.regs.read_endptsetupstat(),
            ));
        self.regs
            .write_endptcomplete(EndpointBitmap::ack_from(self.regs.read_endptcomplete()));
        self.configure_power_detection_interrupts();
        let mut intr = UsbInterrupt::new_with_raw_value(0);
        intr.set_usb_interrupt(true);
        intr.set_usb_error_interrupt(true);
        intr.set_port_change_detect(true);
        intr.set_system_error(true);
        intr.set_reset_received(true);
        intr.set_suspend(true);
        self.regs.write_usbintr(intr);
        self.regs.modify_usbcmd(|mut cmd| {
            cmd.set_run_stop(true);
            cmd
        });

        let runtime = runtime(self.id);
        runtime.set_enabled(EndpointAddress::from_parts(0, Direction::Out), true);
        runtime.set_enabled(EndpointAddress::from_parts(0, Direction::In), true);
        runtime.suspended.store(false, Ordering::Release);
        if let Some(event_bits) = runtime.update_power_present(self.read_power_present()) {
            runtime.push_event(event_bits);
        }
        runtime.bus_waker.wake();
        runtime.control_waker.wake();
        Ok(())
    }

    /// Put the controller into device mode and register a queue-head list for endpoint operation.
    pub fn init_device_mode_with_queue_heads(
        &mut self,
        queue_heads: &mut EndpointQueueHeadList,
    ) -> Result<(), UsbInitError> {
        self.init_device_mode()?;
        let base = queue_heads.base_addr();
        if !base.is_multiple_of(2048) {
            return Err(UsbInitError::QueueHeadAlignment);
        }
        let mut ep_list = zynq7000::usb::EndpointListAddress::new_with_raw_value(0);
        ep_list.set_base_address(u21::new((base >> 11) as u32));
        self.regs.write_endpointlistaddr(ep_list);

        queue_heads
            .endpoint(0, Direction::Out)
            .configure(64, true, true);
        queue_heads
            .endpoint(0, Direction::In)
            .configure(64, false, true);
        queue_heads.endpoint(0, Direction::Out).reset_overlay();
        queue_heads.endpoint(0, Direction::In).reset_overlay();
        self.configure_endpoint(
            EndpointAddress::from_parts(0, Direction::Out),
            EndpointType::Control,
            true,
        );
        self.configure_endpoint(
            EndpointAddress::from_parts(0, Direction::In),
            EndpointType::Control,
            true,
        );
        crate::cache::clean_data_cache_range(
            queue_heads.base_addr(),
            core::mem::size_of::<EndpointQueueHeadList>(),
        )?;
        Ok(())
    }

    /// Stop the controller and mask interrupts.
    pub fn disable(&mut self) {
        self.regs.write_usbintr(UsbInterrupt::DEFAULT);
        self.clear_power_detection_interrupts();
        self.regs.modify_usbcmd(|mut cmd| {
            cmd.set_run_stop(false);
            cmd
        });

        let runtime = runtime(self.id);
        runtime.clear_endpoint_state();
        runtime.suspended.store(false, Ordering::Release);
        runtime.set_power_present(false);
        runtime.bus_waker.wake();
        runtime.control_waker.wake();
    }

    fn configure_power_detection_interrupts(&mut self) {
        let otgsc = self.regs.read_otgsc();
        self.regs
            .write_otgsc(OtgSc::power_detection_irq_write(true, otgsc));
    }

    fn clear_power_detection_interrupts(&mut self) {
        let otgsc = self.regs.read_otgsc();
        self.regs
            .write_otgsc(OtgSc::power_detection_irq_write(false, otgsc));
    }

    fn read_power_present(&self) -> bool {
        power_present_from_otgsc(self.regs.read_otgsc())
    }

    fn reset_via_slcr(&mut self) {
        unsafe {
            Slcr::with(|regs| {
                let assert_reset = match self.id {
                    UsbId::Usb0 => zynq7000::slcr::reset::UsbResetControl::builder()
                        .with_periph1_ref_rst(false)
                        .with_periph0_ref_rst(true)
                        .with_periph1_cpu1x_rst(false)
                        .with_periph0_cpu1x_rst(true)
                        .build(),
                    UsbId::Usb1 => zynq7000::slcr::reset::UsbResetControl::builder()
                        .with_periph1_ref_rst(true)
                        .with_periph0_ref_rst(false)
                        .with_periph1_cpu1x_rst(true)
                        .with_periph0_cpu1x_rst(false)
                        .build(),
                };
                regs.reset_ctrl().write_usb(assert_reset);
                aarch32_cpu::asm::nop();
                regs.reset_ctrl()
                    .write_usb(zynq7000::slcr::reset::UsbResetControl::DEFAULT);
            });
        }
    }

    pub fn configure_endpoint(
        &mut self,
        addr: EndpointAddress,
        ep_type: EndpointType,
        enable: bool,
    ) {
        let idx = addr.index();
        let typ = endpoint_type_bits(ep_type);
        let current = self.regs.read_endptctrl(idx).unwrap_or(0);
        let updated = match addr.direction() {
            Direction::Out => {
                let mut val = current & !(0b11 << EPCTRL_RX_TYPE_SHIFT);
                if enable {
                    val |= EPCTRL_RX_ENABLE
                        | EPCTRL_RX_DATA_TOGGLE_RESET
                        | (typ << EPCTRL_RX_TYPE_SHIFT);
                } else {
                    val &= !EPCTRL_RX_ENABLE;
                }
                val
            }
            Direction::In => {
                let mut val = current & !(0b11 << EPCTRL_TX_TYPE_SHIFT);
                if enable {
                    val |= EPCTRL_TX_ENABLE
                        | EPCTRL_TX_DATA_TOGGLE_RESET
                        | (typ << EPCTRL_TX_TYPE_SHIFT);
                } else {
                    val &= !EPCTRL_TX_ENABLE;
                }
                val
            }
        };
        let _ = self.regs.write_endptctrl(idx, updated);
        runtime(self.id).set_enabled(addr, enable);
    }

    pub fn poll_interrupt_result(&mut self) -> InterruptResult {
        poll_interrupt_result_regs(self.id, &mut self.regs)
    }

    pub fn wait_for_bus_reset_blocking(&mut self) -> Result<(), UsbInitError> {
        loop {
            let result = self.poll_interrupt_result();
            if result.reset {
                return Ok(());
            }
        }
    }

    pub fn read_setup_packet(
        &mut self,
        queue_heads: &mut EndpointQueueHeadList,
        ep_index: usize,
    ) -> Result<Option<[u8; SETUP_PACKET_SIZE]>, UsbInitError> {
        let setup_status = self.regs.read_endptsetupstat().setup_endpoints();
        let bit = 1u16 << ep_index;
        if setup_status & bit == 0 {
            return Ok(None);
        }

        let packet = loop {
            self.regs.modify_usbcmd(|mut cmd| {
                cmd.set_setup_tripwire(true);
                cmd
            });
            crate::cache::invalidate_data_cache_range(
                queue_heads.endpoint(ep_index, Direction::Out) as *const EndpointQueueHead as u32,
                core::mem::size_of::<EndpointQueueHead>(),
            )?;

            let words = queue_heads.endpoint(ep_index, Direction::Out).setup_words();
            let packet = [
                (words[0] & 0xff) as u8,
                ((words[0] >> 8) & 0xff) as u8,
                ((words[0] >> 16) & 0xff) as u8,
                ((words[0] >> 24) & 0xff) as u8,
                (words[1] & 0xff) as u8,
                ((words[1] >> 8) & 0xff) as u8,
                ((words[1] >> 16) & 0xff) as u8,
                ((words[1] >> 24) & 0xff) as u8,
            ];
            if self.regs.read_usbcmd().setup_tripwire() {
                break packet;
            }
        };

        self.regs
            .write_endptsetupstat(EndpointSetupStatus::ack_mask(bit));
        self.regs.modify_usbcmd(|mut cmd| {
            cmd.set_setup_tripwire(false);
            cmd
        });
        Ok(Some(packet))
    }

    pub fn prime_transfer(
        &mut self,
        queue_heads: &mut EndpointQueueHeadList,
        addr: EndpointAddress,
        dtd: &mut TransferDescriptor,
        buf_addr: u32,
        len: usize,
        interrupt_on_complete: bool,
    ) -> Result<(), UsbInitError> {
        dtd.configure(buf_addr, len, interrupt_on_complete);
        queue_heads
            .endpoint(addr.index(), addr.direction())
            .attach_dtd(dtd);
        crate::cache::clean_data_cache_range(
            (dtd as *mut TransferDescriptor) as u32,
            core::mem::size_of::<TransferDescriptor>(),
        )?;
        crate::cache::clean_data_cache_range(
            queue_heads.base_addr(),
            core::mem::size_of::<EndpointQueueHeadList>(),
        )?;
        self.regs.write_endptprime(endpoint_bitmap_for(addr));
        Ok(())
    }

    pub fn prepare_out_buffer<const N: usize>(
        &mut self,
        buffer: &mut AlignedUsbBuffer<N>,
    ) -> Result<(), UsbInitError> {
        crate::cache::invalidate_data_cache_range(buffer.as_mut_ptr() as u32, N)?;
        Ok(())
    }

    pub fn prepare_in_buffer<const N: usize>(
        &mut self,
        buffer: &AlignedUsbBuffer<N>,
    ) -> Result<(), UsbInitError> {
        crate::cache::clean_data_cache_range(buffer.as_ptr() as u32, N)?;
        Ok(())
    }

    pub fn finish_out_buffer<const N: usize>(
        &mut self,
        buffer: &mut AlignedUsbBuffer<N>,
    ) -> Result<(), UsbInitError> {
        crate::cache::invalidate_data_cache_range(buffer.as_mut_ptr() as u32, N)?;
        Ok(())
    }

    pub fn poll_transfer_complete(
        &mut self,
        addr: EndpointAddress,
        dtd: &TransferDescriptor,
    ) -> bool {
        let bit = endpoint_mask(addr);
        let done = runtime(self.id).take_completed(addr) || {
            let complete = self.regs.read_endptcomplete();
            let done = match addr.direction() {
                Direction::Out => complete.rx() & bit != 0,
                Direction::In => complete.tx() & bit != 0,
            };
            if done {
                self.regs.write_endptcomplete(endpoint_bitmap_for(addr));
            }
            done
        };
        if done {
            let _ = crate::cache::invalidate_data_cache_range(
                (dtd as *const TransferDescriptor) as u32,
                core::mem::size_of::<TransferDescriptor>(),
            );
        }
        done
    }
}

impl<'a> UsbDevice<'a> {
    pub fn new(controller: UsbController, resources: &'a mut UsbResources) -> Self {
        Self {
            controller,
            resources,
        }
    }

    pub const fn id(&self) -> UsbId {
        self.controller.id()
    }

    pub fn start_device_mode(&mut self) -> Result<(), UsbError> {
        self.controller.init_device_mode()?;
        self.on_bus_reset()?;
        Ok(())
    }

    pub fn disable(&mut self) {
        self.controller.disable();
        self.resources.reset_state();
    }

    pub fn poll(&mut self) -> InterruptResult {
        self.controller.poll_interrupt_result()
    }

    pub fn on_bus_reset(&mut self) -> Result<(), UsbError> {
        self.resources.reset_state();
        runtime(self.id()).clear_endpoint_state();
        self.register_queue_heads()?;
        self.configure_endpoint(ep0_addr(Direction::Out), EndpointType::Control, 64, true)?;
        self.configure_endpoint(ep0_addr(Direction::In), EndpointType::Control, 64, true)?;
        self.set_stalled(ep0_addr(Direction::Out), false)?;
        self.set_stalled(ep0_addr(Direction::In), false)?;
        self.clean_all_queue_heads()?;
        Ok(())
    }

    pub fn reset_endpoint_state(&mut self) -> Result<(), UsbError> {
        self.on_bus_reset()
    }

    pub fn configure_endpoint(
        &mut self,
        addr: EndpointAddress,
        ep_type: EndpointType,
        max_packet_size: u16,
        enable: bool,
    ) -> Result<(), UsbError> {
        self.validate_endpoint(addr)?;

        if enable {
            self.resources
                .queue_heads
                .endpoint(addr.index(), addr.direction())
                .configure(
                    max_packet_size,
                    addr.index() == 0 && addr.direction() == Direction::Out,
                    true,
                );
            self.resources
                .queue_heads
                .endpoint(addr.index(), addr.direction())
                .reset_overlay();
            self.clean_queue_head(addr)?;
        } else {
            self.resources.set_requested(addr, 0);
        }
        self.controller.configure_endpoint(addr, ep_type, enable);
        if !enable {
            self.flush_endpoint(addr)?;
        }
        Ok(())
    }

    pub fn set_stalled(&mut self, addr: EndpointAddress, stalled: bool) -> Result<(), UsbError> {
        self.validate_endpoint(addr)?;
        let current = self
            .controller
            .regs
            .read_endptctrl(addr.index())
            .map_err(|_| UsbError::InvalidEndpoint(addr.index()))?;
        let updated = match addr.direction() {
            Direction::Out => {
                if stalled {
                    current | EPCTRL_RX_STALL
                } else {
                    current & !EPCTRL_RX_STALL
                }
            }
            Direction::In => {
                if stalled {
                    current | EPCTRL_TX_STALL
                } else {
                    current & !EPCTRL_TX_STALL
                }
            }
        };
        let _ = self.controller.regs.write_endptctrl(addr.index(), updated);
        runtime(self.id()).set_stalled(addr, stalled);
        runtime(self.id()).control_waker.wake();
        runtime(self.id()).data_in_waker.wake();
        runtime(self.id()).data_out_waker.wake();
        Ok(())
    }

    pub fn flush_endpoint(&mut self, addr: EndpointAddress) -> Result<(), UsbError> {
        self.validate_endpoint(addr)?;
        let mask = endpoint_bitmap_for(addr);
        self.controller.regs.write_endptflush(mask);
        for _ in 0..ENDPOINT_FLUSH_TIMEOUT_ITERS {
            if !endpoint_bitmap_is_set(self.controller.regs.read_endptprime(), addr)
                && !endpoint_bitmap_is_set(self.controller.regs.read_endptflush(), addr)
                && !endpoint_bitmap_is_set(self.controller.regs.read_endptstatus(), addr)
            {
                return Ok(());
            }
        }
        Err(UsbError::EndpointFlushTimedOut(addr))
    }

    pub fn read_setup_packet(&mut self) -> Result<Option<[u8; SETUP_PACKET_SIZE]>, UsbError> {
        self.controller
            .read_setup_packet(&mut self.resources.queue_heads, 0)
            .map_err(Into::into)
    }

    pub fn prime_out<const N: usize>(
        &mut self,
        addr: EndpointAddress,
        buffer: &mut AlignedUsbBuffer<N>,
        len: usize,
    ) -> Result<(), UsbError> {
        self.validate_transfer(addr, N, len)?;
        if addr.direction() != Direction::Out {
            return Err(UsbError::InvalidEndpoint(addr.index()));
        }
        self.ensure_endpoint_idle(addr)?;
        self.controller.prepare_out_buffer(buffer)?;
        self.resources.set_requested(addr, len);
        self.prime_transfer_inner(addr, buffer.as_mut_ptr() as u32, len)
    }

    pub fn prime_in<const N: usize>(
        &mut self,
        addr: EndpointAddress,
        buffer: &AlignedUsbBuffer<N>,
        len: usize,
    ) -> Result<(), UsbError> {
        self.validate_transfer(addr, N, len)?;
        if addr.direction() != Direction::In {
            return Err(UsbError::InvalidEndpoint(addr.index()));
        }
        self.ensure_endpoint_idle(addr)?;
        self.controller.prepare_in_buffer(buffer)?;
        self.resources.set_requested(addr, len);
        self.prime_transfer_inner(addr, buffer.as_ptr() as u32, len)
    }

    pub fn take_transfer_complete(
        &mut self,
        addr: EndpointAddress,
    ) -> Result<Option<TransferReport>, UsbError> {
        self.validate_endpoint(addr)?;
        let dtd = self.resources.dtd(addr);
        if !self.controller.poll_transfer_complete(addr, dtd) {
            return Ok(None);
        }
        let requested = self.resources.requested(addr);
        let token = dtd.token.get();
        let report = TransferReport {
            requested_bytes: requested,
            actual_bytes: dtd.actual_bytes_transferred(requested),
            token,
        };
        self.resources.set_requested(addr, 0);
        if report.has_error() {
            return Err(UsbError::TransferFailed { addr, token });
        }
        Ok(Some(report))
    }

    pub fn finish_out<const N: usize>(
        &mut self,
        buffer: &mut AlignedUsbBuffer<N>,
    ) -> Result<(), UsbError> {
        self.controller.finish_out_buffer(buffer)?;
        Ok(())
    }

    pub fn prime_ep0_in(&mut self, data: &[u8], requested_len: usize) -> Result<usize, UsbError> {
        let len = core::cmp::min(
            CONTROL_BUFFER_SIZE,
            core::cmp::min(data.len(), requested_len),
        );
        self.resources.ep0_in_buffer.0[..len].copy_from_slice(&data[..len]);
        self.ensure_endpoint_idle(ep0_addr(Direction::In))?;
        self.controller
            .prepare_in_buffer(&self.resources.ep0_in_buffer)?;
        self.resources.set_requested(ep0_addr(Direction::In), len);
        self.prime_transfer_inner(
            ep0_addr(Direction::In),
            self.resources.ep0_in_buffer.as_ptr() as u32,
            len,
        )?;
        Ok(len)
    }

    pub fn prime_ep0_in_status(&mut self) -> Result<(), UsbError> {
        self.ensure_endpoint_idle(ep0_addr(Direction::In))?;
        self.controller
            .prepare_in_buffer(&self.resources.ep0_in_buffer)?;
        self.resources.set_requested(ep0_addr(Direction::In), 0);
        self.prime_transfer_inner(
            ep0_addr(Direction::In),
            self.resources.ep0_in_buffer.as_ptr() as u32,
            0,
        )
    }

    pub fn prime_ep0_out_status(&mut self) -> Result<(), UsbError> {
        self.ensure_endpoint_idle(ep0_addr(Direction::Out))?;
        self.controller
            .prepare_out_buffer(&mut self.resources.ep0_out_buffer)?;
        self.resources.set_requested(ep0_addr(Direction::Out), 0);
        let buf_addr = self.resources.ep0_out_buffer.as_mut_ptr() as u32;
        self.prime_transfer_inner(ep0_addr(Direction::Out), buf_addr, 0)
    }

    pub fn prime_ep0_out_data(&mut self, expected_len: usize) -> Result<usize, UsbError> {
        let len = core::cmp::min(expected_len, CONTROL_BUFFER_SIZE);
        self.ensure_endpoint_idle(ep0_addr(Direction::Out))?;
        self.controller
            .prepare_out_buffer(&mut self.resources.ep0_out_buffer)?;
        self.resources.set_requested(ep0_addr(Direction::Out), len);
        let buf_addr = self.resources.ep0_out_buffer.as_mut_ptr() as u32;
        self.prime_transfer_inner(ep0_addr(Direction::Out), buf_addr, len)?;
        Ok(len)
    }

    pub fn take_ep0_out_data(&mut self, dst: &mut [u8]) -> Result<Option<usize>, UsbError> {
        let addr = ep0_addr(Direction::Out);
        let Some(report) = self.take_transfer_complete(addr)? else {
            return Ok(None);
        };
        {
            let controller = &mut self.controller;
            let buffer = &mut self.resources.ep0_out_buffer;
            controller.finish_out_buffer(buffer)?;
        }
        if report.actual_bytes > dst.len() {
            return Err(UsbError::BufferTooSmall {
                requested: report.actual_bytes,
                buffer_len: dst.len(),
            });
        }
        dst[..report.actual_bytes]
            .copy_from_slice(&self.resources.ep0_out_buffer.0[..report.actual_bytes]);
        Ok(Some(report.actual_bytes))
    }

    pub fn stall_ep0(&mut self) -> Result<(), UsbError> {
        self.set_stalled(ep0_addr(Direction::Out), true)?;
        self.set_stalled(ep0_addr(Direction::In), true)?;
        Ok(())
    }

    pub fn abort_ep0_state(&mut self) -> Result<(), UsbError> {
        let ep0_out = ep0_addr(Direction::Out);
        let ep0_in = ep0_addr(Direction::In);
        self.flush_endpoint(ep0_out)?;
        self.flush_endpoint(ep0_in)?;
        self.resources.clear_ep0_transfer_state();
        runtime(self.id()).clear_completed(ep0_out);
        runtime(self.id()).clear_completed(ep0_in);
        self.set_stalled(ep0_out, false)?;
        self.set_stalled(ep0_in, false)?;
        self.clean_queue_head(ep0_out)?;
        self.clean_queue_head(ep0_in)?;
        self.clean_dtd(ep0_out)?;
        self.clean_dtd(ep0_in)?;
        Ok(())
    }

    pub fn arm_address_after_status(&mut self, addr: u8) {
        self.controller
            .regs
            .write_deviceaddr(staged_device_address(addr));
    }

    fn register_queue_heads(&mut self) -> Result<(), UsbError> {
        let base = self.resources.queue_heads.base_addr();
        if !base.is_multiple_of(2048) {
            return Err(UsbInitError::QueueHeadAlignment.into());
        }
        let mut ep_list = zynq7000::usb::EndpointListAddress::new_with_raw_value(0);
        ep_list.set_base_address(u21::new((base >> 11) as u32));
        self.controller.regs.write_endpointlistaddr(ep_list);
        Ok(())
    }

    fn validate_endpoint(&self, addr: EndpointAddress) -> Result<(), UsbError> {
        if addr.index() >= MAX_ENDPOINTS {
            return Err(UsbError::InvalidEndpoint(addr.index()));
        }
        Ok(())
    }

    fn validate_transfer(
        &self,
        addr: EndpointAddress,
        buffer_len: usize,
        requested: usize,
    ) -> Result<(), UsbError> {
        self.validate_endpoint(addr)?;
        if requested > buffer_len {
            return Err(UsbError::BufferTooSmall {
                requested,
                buffer_len,
            });
        }
        if requested > MAX_DTD_TRANSFER_BYTES {
            return Err(UsbError::TransferTooLarge(requested));
        }
        Ok(())
    }

    fn ensure_endpoint_idle(&mut self, addr: EndpointAddress) -> Result<(), UsbError> {
        self.validate_endpoint(addr)?;
        let dtd = self.resources.dtd(addr);
        if dtd.is_active() {
            return Err(UsbError::EndpointBusy(addr));
        }
        let status = self.controller.regs.read_endptstatus();
        let active = match addr.direction() {
            Direction::Out => status.rx() & endpoint_mask(addr) != 0,
            Direction::In => status.tx() & endpoint_mask(addr) != 0,
        };
        if active {
            return Err(UsbError::EndpointBusy(addr));
        }
        Ok(())
    }

    fn prime_transfer_inner(
        &mut self,
        addr: EndpointAddress,
        buf_addr: u32,
        len: usize,
    ) -> Result<(), UsbError> {
        {
            let dtd = self.resources.dtd_mut(addr);
            dtd.configure(buf_addr, len, true);
        }
        self.attach_dtd_with_tripwire(addr)?;
        let dtd = self.resources.dtd(addr);
        crate::cache::clean_data_cache_range(
            (dtd as *const TransferDescriptor) as u32,
            core::mem::size_of::<TransferDescriptor>(),
        )?;
        self.clean_queue_head(addr)?;
        self.controller
            .regs
            .write_endptprime(endpoint_bitmap_for(addr));
        Ok(())
    }

    fn attach_dtd_with_tripwire(&mut self, addr: EndpointAddress) -> Result<(), UsbError> {
        loop {
            self.controller.regs.modify_usbcmd(|mut cmd| {
                cmd.set_add_dtd_tripwire(true);
                cmd
            });
            self.resources
                .queue_heads
                .endpoint(addr.index(), addr.direction())
                .attach_dtd(self.resources.dtd(addr));
            self.clean_queue_head(addr)?;
            if self.controller.regs.read_usbcmd().add_dtd_tripwire() {
                break;
            }
        }
        self.controller.regs.modify_usbcmd(|mut cmd| {
            cmd.set_add_dtd_tripwire(false);
            cmd
        });
        Ok(())
    }

    fn clean_queue_head(&mut self, addr: EndpointAddress) -> Result<(), UsbError> {
        crate::cache::clean_data_cache_range(
            self.resources
                .queue_heads
                .endpoint(addr.index(), addr.direction()) as *const EndpointQueueHead
                as u32,
            core::mem::size_of::<EndpointQueueHead>(),
        )?;
        Ok(())
    }

    fn clean_dtd(&mut self, addr: EndpointAddress) -> Result<(), UsbError> {
        crate::cache::clean_data_cache_range(
            self.resources.dtd(addr) as *const TransferDescriptor as u32,
            core::mem::size_of::<TransferDescriptor>(),
        )?;
        Ok(())
    }

    fn clean_all_queue_heads(&mut self) -> Result<(), UsbError> {
        crate::cache::clean_data_cache_range(
            self.resources.queue_heads.base_addr(),
            core::mem::size_of::<EndpointQueueHeadList>(),
        )?;
        Ok(())
    }
}

/// IRQ entry point for the USB HAL and Embassy wakeups.
pub fn on_interrupt(id: UsbId) -> InterruptResult {
    with_regs(id, |regs| poll_interrupt_result_regs(id, regs))
}

fn poll_interrupt_result_regs(id: UsbId, regs: &mut MmioRegisters<'static>) -> InterruptResult {
    let status = regs.read_usbsts();
    let setup = regs.read_endptsetupstat();
    let complete = regs.read_endptcomplete();
    let otgsc = regs.read_otgsc();
    let mut result = InterruptResult {
        usb_interrupt: status.usb_interrupt(),
        usb_error: status.usb_error_interrupt() || status.system_error(),
        port_change: status.port_change_detect(),
        reset: status.reset_received(),
        suspend: status.suspend(),
        resume: false,
        setup_endpoints: setup.setup_endpoints(),
        completed_out_endpoints: complete.rx(),
        completed_in_endpoints: complete.tx(),
    };

    let rt = runtime(id);
    if result.reset {
        rt.clear_endpoint_state();
        rt.note_reset();
        rt.suspended.store(false, Ordering::Release);
        rt.push_event(EVENT_RESET);
    }
    if result.completed_out_endpoints != 0 {
        rt.push_completed(Direction::Out, result.completed_out_endpoints);
    }
    if result.completed_in_endpoints != 0 {
        rt.push_completed(Direction::In, result.completed_in_endpoints);
    }
    if result.suspend {
        rt.suspended.store(true, Ordering::Release);
        rt.push_event(EVENT_SUSPEND);
    }
    if result.port_change {
        let port = regs.read_portsc1();
        let connected = port.current_connect_status();
        let was_suspended = rt.suspended.load(Ordering::Acquire);
        if let Some(event_bits) = port_change_event_bits(was_suspended, connected) {
            rt.suspended.store(false, Ordering::Release);
            rt.push_event(event_bits);
            result.resume = true;
        } else if !connected {
            rt.suspended.store(false, Ordering::Release);
        }
        acknowledge_port_change(regs, port);
    }
    if let Some(event_bits) = rt.update_power_present(power_present_from_otgsc(otgsc)) {
        rt.push_event(event_bits);
    }
    regs.write_usbsts(UsbStatus::ack_from(status));
    regs.write_endptcomplete(EndpointBitmap::ack_from(complete));
    regs.write_otgsc(OtgSc::power_detection_irq_write(
        power_detection_interrupts_enabled(otgsc),
        otgsc,
    ));
    rt.bus_waker.wake();
    rt.control_waker.wake();
    rt.data_in_waker.wake();
    rt.data_out_waker.wake();
    result
}

fn with_regs<R>(id: UsbId, f: impl FnOnce(&mut MmioRegisters<'static>) -> R) -> R {
    let mut regs = unsafe {
        match id {
            UsbId::Usb0 => zynq7000::usb::Registers::new_mmio_fixed_0(),
            UsbId::Usb1 => zynq7000::usb::Registers::new_mmio_fixed_1(),
        }
    };
    f(&mut regs)
}

fn fresh_controller(id: UsbId) -> UsbController {
    let regs = unsafe {
        match id {
            UsbId::Usb0 => zynq7000::usb::Registers::new_mmio_fixed_0(),
            UsbId::Usb1 => zynq7000::usb::Registers::new_mmio_fixed_1(),
        }
    };
    UsbController::new(id, regs)
}

fn endpoint_mask(addr: EndpointAddress) -> u16 {
    1u16 << addr.index()
}

fn endpoint_bitmap_for(addr: EndpointAddress) -> EndpointBitmap {
    let mut bitmap = EndpointBitmap::new_with_raw_value(0);
    match addr.direction() {
        Direction::Out => bitmap.set_rx(endpoint_mask(addr)),
        Direction::In => bitmap.set_tx(endpoint_mask(addr)),
    }
    bitmap
}

fn endpoint_bitmap_is_set(bitmap: EndpointBitmap, addr: EndpointAddress) -> bool {
    match addr.direction() {
        Direction::Out => bitmap.rx() & endpoint_mask(addr) != 0,
        Direction::In => bitmap.tx() & endpoint_mask(addr) != 0,
    }
}

fn ep0_addr(direction: Direction) -> EndpointAddress {
    EndpointAddress::from_parts(0, direction)
}

fn staged_device_address(addr: u8) -> DeviceAddress {
    DeviceAddress::builder()
        .with_usb_address(u7::new(addr))
        .with_address_advance(true)
        .build()
}

const fn power_present_from_otgsc(otgsc: OtgSc) -> bool {
    otgsc.b_session_valid() || otgsc.a_session_valid() || otgsc.a_vbus_valid()
}

const fn power_detection_interrupts_enabled(otgsc: OtgSc) -> bool {
    otgsc.b_session_end_interrupt_enable()
}

const fn power_event_bits_for_transition(previous: bool, current: bool) -> Option<u32> {
    match (previous, current) {
        (false, true) => Some(EVENT_POWER_DETECTED),
        (true, false) => Some(EVENT_POWER_REMOVED),
        _ => None,
    }
}

fn acknowledge_port_change(regs: &mut MmioRegisters<'static>, port: PortSc1) {
    if !port.connect_status_change()
        && !port.port_enable_disable_change()
        && !port.overcurrent_change()
    {
        return;
    }
    regs.write_portsc1(PortSc1::ack_changes_from(port));
}

const fn port_change_event_bits(was_suspended: bool, connected: bool) -> Option<u32> {
    if was_suspended && connected {
        Some(EVENT_RESUME)
    } else {
        None
    }
}

const fn queue_head_index(ep_index: usize, dir: Direction) -> usize {
    ep_index * 2
        + match dir {
            Direction::Out => 0,
            Direction::In => 1,
        }
}

const fn endpoint_type_bits(ep_type: EndpointType) -> u32 {
    match ep_type {
        EndpointType::Control => 0,
        EndpointType::Isochronous => 1,
        EndpointType::Bulk => 2,
        EndpointType::Interrupt => 3,
    }
}

struct ControllerRuntime {
    bus_waker: AtomicWaker,
    control_waker: AtomicWaker,
    data_in_waker: AtomicWaker,
    data_out_waker: AtomicWaker,
    event_flags: AtomicU32,
    enabled_out: AtomicU16,
    enabled_in: AtomicU16,
    stalled_out: AtomicU16,
    stalled_in: AtomicU16,
    completed_out: AtomicU16,
    completed_in: AtomicU16,
    reset_count: AtomicU32,
    suspended: AtomicBool,
    power_present: AtomicBool,
    failed: AtomicBool,
}

impl ControllerRuntime {
    const fn new() -> Self {
        Self {
            bus_waker: AtomicWaker::new(),
            control_waker: AtomicWaker::new(),
            data_in_waker: AtomicWaker::new(),
            data_out_waker: AtomicWaker::new(),
            event_flags: AtomicU32::new(0),
            enabled_out: AtomicU16::new(0),
            enabled_in: AtomicU16::new(0),
            stalled_out: AtomicU16::new(0),
            stalled_in: AtomicU16::new(0),
            completed_out: AtomicU16::new(0),
            completed_in: AtomicU16::new(0),
            reset_count: AtomicU32::new(0),
            suspended: AtomicBool::new(false),
            power_present: AtomicBool::new(false),
            failed: AtomicBool::new(false),
        }
    }

    fn push_event(&self, bits: u32) {
        self.event_flags.fetch_or(bits, Ordering::AcqRel);
    }

    fn set_enabled(&self, addr: EndpointAddress, enabled: bool) {
        let mask = endpoint_mask(addr);
        let target = match addr.direction() {
            Direction::Out => &self.enabled_out,
            Direction::In => &self.enabled_in,
        };
        if enabled {
            target.fetch_or(mask, Ordering::AcqRel);
        } else {
            target.fetch_and(!mask, Ordering::AcqRel);
        }
    }

    fn is_enabled(&self, addr: EndpointAddress) -> bool {
        let mask = endpoint_mask(addr);
        let target = match addr.direction() {
            Direction::Out => &self.enabled_out,
            Direction::In => &self.enabled_in,
        };
        target.load(Ordering::Acquire) & mask != 0
    }

    fn set_stalled(&self, addr: EndpointAddress, stalled: bool) {
        let mask = endpoint_mask(addr);
        let target = match addr.direction() {
            Direction::Out => &self.stalled_out,
            Direction::In => &self.stalled_in,
        };
        if stalled {
            target.fetch_or(mask, Ordering::AcqRel);
        } else {
            target.fetch_and(!mask, Ordering::AcqRel);
        }
    }

    fn is_stalled(&self, addr: EndpointAddress) -> bool {
        let mask = endpoint_mask(addr);
        let target = match addr.direction() {
            Direction::Out => &self.stalled_out,
            Direction::In => &self.stalled_in,
        };
        target.load(Ordering::Acquire) & mask != 0
    }

    fn clear_endpoint_state(&self) {
        self.enabled_out.store(0, Ordering::Release);
        self.enabled_in.store(0, Ordering::Release);
        self.stalled_out.store(0, Ordering::Release);
        self.stalled_in.store(0, Ordering::Release);
        self.completed_out.store(0, Ordering::Release);
        self.completed_in.store(0, Ordering::Release);
    }

    fn note_reset(&self) {
        self.reset_count.fetch_add(1, Ordering::AcqRel);
    }

    fn update_power_present(&self, current: bool) -> Option<u32> {
        let previous = self.power_present.swap(current, Ordering::AcqRel);
        power_event_bits_for_transition(previous, current)
    }

    fn set_power_present(&self, current: bool) {
        self.power_present.store(current, Ordering::Release);
    }

    fn set_failed(&self, failed: bool) {
        self.failed.store(failed, Ordering::Release);
    }

    fn has_failed(&self) -> bool {
        self.failed.load(Ordering::Acquire)
    }

    fn reset_count(&self) -> u32 {
        self.reset_count.load(Ordering::Acquire)
    }

    fn push_completed(&self, dir: Direction, mask: u16) {
        let target = match dir {
            Direction::Out => &self.completed_out,
            Direction::In => &self.completed_in,
        };
        target.fetch_or(mask, Ordering::AcqRel);
    }

    fn take_completed(&self, addr: EndpointAddress) -> bool {
        let mask = endpoint_mask(addr);
        let target = match addr.direction() {
            Direction::Out => &self.completed_out,
            Direction::In => &self.completed_in,
        };
        loop {
            let current = target.load(Ordering::Acquire);
            if current & mask == 0 {
                return false;
            }
            if target
                .compare_exchange(
                    current,
                    current & !mask,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
            {
                return true;
            }
        }
    }

    fn clear_completed(&self, addr: EndpointAddress) {
        let mask = endpoint_mask(addr);
        let target = match addr.direction() {
            Direction::Out => &self.completed_out,
            Direction::In => &self.completed_in,
        };
        target.fetch_and(!mask, Ordering::AcqRel);
    }

    fn take_event(&self) -> Option<Event> {
        loop {
            let current = self.event_flags.load(Ordering::Acquire);
            if current == 0 {
                return None;
            }
            let (bit, event) = if current & EVENT_RESET != 0 {
                (EVENT_RESET, Event::Reset)
            } else if current & EVENT_SUSPEND != 0 {
                (EVENT_SUSPEND, Event::Suspend)
            } else if current & EVENT_RESUME != 0 {
                (EVENT_RESUME, Event::Resume)
            } else if current & EVENT_POWER_DETECTED != 0 {
                (EVENT_POWER_DETECTED, Event::PowerDetected)
            } else if current & EVENT_POWER_REMOVED != 0 {
                (EVENT_POWER_REMOVED, Event::PowerRemoved)
            } else {
                return None;
            };
            if self
                .event_flags
                .compare_exchange(current, current & !bit, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return Some(event);
            }
        }
    }
}

static USB0_RUNTIME: ControllerRuntime = ControllerRuntime::new();
static USB1_RUNTIME: ControllerRuntime = ControllerRuntime::new();

fn runtime(id: UsbId) -> &'static ControllerRuntime {
    match id.index() {
        0 => &USB0_RUNTIME,
        _ => &USB1_RUNTIME,
    }
}

pub mod embassy {
    //! Embassy USB driver surface backed by the Phase 2 Zynq USB HAL.

    use core::cell::RefCell;

    use critical_section::Mutex;

    use super::*;

    struct SharedState {
        resources: UsbResources,
        out_info: [Option<EndpointInfo>; MAX_ENDPOINTS],
        in_info: [Option<EndpointInfo>; MAX_ENDPOINTS],
        out_buffers: [AlignedUsbBuffer<CONTROL_BUFFER_SIZE>; MAX_ENDPOINTS],
        in_buffers: [AlignedUsbBuffer<CONTROL_BUFFER_SIZE>; MAX_ENDPOINTS],
        setup_packet: Option<[u8; 8]>,
    }

    impl SharedState {
        const fn new() -> Self {
            Self {
                resources: UsbResources::new(),
                out_info: [None; MAX_ENDPOINTS],
                in_info: [None; MAX_ENDPOINTS],
                out_buffers: [const { AlignedUsbBuffer::new() }; MAX_ENDPOINTS],
                in_buffers: [const { AlignedUsbBuffer::new() }; MAX_ENDPOINTS],
                setup_packet: None,
            }
        }

        fn store_allocations(
            &mut self,
            out_info: [Option<EndpointInfo>; MAX_ENDPOINTS],
            in_info: [Option<EndpointInfo>; MAX_ENDPOINTS],
        ) {
            self.resources = UsbResources::new();
            self.out_info = out_info;
            self.in_info = in_info;
            self.clear_cached_setup();
        }

        fn clear_cached_setup(&mut self) {
            self.setup_packet = None;
        }

        fn start_device(&mut self, id: UsbId) -> Result<(), UsbError> {
            self.clear_cached_setup();
            let mut device = fresh_controller(id).into_device(&mut self.resources);
            device.start_device_mode()
        }

        fn reset_after_bus_reset(&mut self, id: UsbId) -> Result<(), UsbError> {
            self.clear_cached_setup();
            let mut device = fresh_controller(id).into_device(&mut self.resources);
            device.on_bus_reset()
        }
    }

    static USB0_STATE: Mutex<RefCell<SharedState>> = Mutex::new(RefCell::new(SharedState::new()));
    static USB1_STATE: Mutex<RefCell<SharedState>> = Mutex::new(RefCell::new(SharedState::new()));

    fn shared_state(id: UsbId) -> &'static Mutex<RefCell<SharedState>> {
        match id {
            UsbId::Usb0 => &USB0_STATE,
            UsbId::Usb1 => &USB1_STATE,
        }
    }

    fn with_shared_state<R>(id: UsbId, f: impl FnOnce(&mut SharedState) -> R) -> R {
        critical_section::with(|cs| {
            let mut state = shared_state(id).borrow_ref_mut(cs);
            f(&mut state)
        })
    }

    fn map_usb_error(err: UsbError) -> EndpointError {
        match err {
            UsbError::BufferTooSmall { .. } | UsbError::TransferTooLarge(_) => {
                EndpointError::BufferOverflow
            }
            _ => EndpointError::Disabled,
        }
    }

    fn fail_driver(id: UsbId) {
        let rt = runtime(id);
        rt.set_failed(true);
        rt.clear_endpoint_state();
        rt.bus_waker.wake();
        rt.control_waker.wake();
        rt.data_in_waker.wake();
        rt.data_out_waker.wake();
    }

    fn clear_driver_failure(id: UsbId) {
        runtime(id).set_failed(false);
    }

    fn maybe_capture_setup(state: &mut SharedState, id: UsbId) -> Result<bool, UsbError> {
        if state.setup_packet.is_some() {
            return Ok(true);
        }
        let mut device = fresh_controller(id).into_device(&mut state.resources);
        if let Some(packet) = device.read_setup_packet()? {
            device.abort_ep0_state()?;
            state.setup_packet = Some(packet);
            return Ok(true);
        }
        Ok(false)
    }

    fn wait_for_reset(rt: &ControllerRuntime, baseline: u32) -> Result<(), EndpointError> {
        if rt.has_failed() || rt.reset_count() != baseline {
            Err(EndpointError::Disabled)
        } else {
            Ok(())
        }
    }

    /// Embassy interrupt handler for a typed USB controller instance.
    pub struct InterruptHandler<T: super::Instance>(PhantomData<T>);

    impl<T: super::Instance> crate::interrupt::typelevel::Handler<T::Interrupt>
        for InterruptHandler<T>
    {
        unsafe fn on_interrupt() {
            super::on_interrupt(T::ID);
        }
    }

    /// Embassy USB driver entry point.
    pub struct Driver<'d> {
        id: UsbId,
        allocated_out: u16,
        allocated_in: u16,
        out_info: [Option<EndpointInfo>; MAX_ENDPOINTS],
        in_info: [Option<EndpointInfo>; MAX_ENDPOINTS],
        _phantom: PhantomData<&'d mut ()>,
    }

    impl<'d> Driver<'d> {
        /// Create a new Embassy USB driver from a HAL controller wrapper.
        pub fn new<T: super::Instance>(
            controller: UsbController,
            _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        ) -> Self {
            assert_eq!(controller.id(), T::ID);
            Self {
                id: controller.id(),
                allocated_out: 0b1,
                allocated_in: 0b1,
                out_info: [None; MAX_ENDPOINTS],
                in_info: [None; MAX_ENDPOINTS],
                _phantom: PhantomData,
            }
        }

        fn alloc_endpoint(
            allocated: &mut u16,
            infos: &mut [Option<EndpointInfo>; MAX_ENDPOINTS],
            dir: Direction,
            ep_type: EndpointType,
            ep_addr: Option<EndpointAddress>,
            max_packet_size: u16,
            interval_ms: u8,
        ) -> Result<EndpointHandle, EndpointAllocError> {
            if ep_type == EndpointType::Isochronous
                || max_packet_size as usize > CONTROL_BUFFER_SIZE
            {
                return Err(EndpointAllocError);
            }
            let addr = if let Some(addr) = ep_addr {
                if addr.direction() != dir
                    || addr.index() >= MAX_ENDPOINTS
                    || (*allocated & endpoint_mask(addr)) != 0
                {
                    return Err(EndpointAllocError);
                }
                addr
            } else {
                let mut chosen = None;
                for index in 1..MAX_ENDPOINTS {
                    let candidate = EndpointAddress::from_parts(index, dir);
                    if (*allocated & endpoint_mask(candidate)) == 0 {
                        chosen = Some(candidate);
                        break;
                    }
                }
                chosen.ok_or(EndpointAllocError)?
            };
            *allocated |= endpoint_mask(addr);
            let info = EndpointInfo {
                addr,
                ep_type,
                max_packet_size,
                interval_ms,
            };
            infos[addr.index()] = Some(info);
            Ok(EndpointHandle { info })
        }
    }

    impl<'d> embassy_usb_driver::Driver<'d> for Driver<'d> {
        type EndpointOut = EndpointOut;
        type EndpointIn = EndpointIn;
        type ControlPipe = Control;
        type Bus = UsbBus;

        fn alloc_endpoint_out(
            &mut self,
            ep_type: EndpointType,
            ep_addr: Option<EndpointAddress>,
            max_packet_size: u16,
            interval_ms: u8,
        ) -> Result<Self::EndpointOut, EndpointAllocError> {
            let ep = Self::alloc_endpoint(
                &mut self.allocated_out,
                &mut self.out_info,
                Direction::Out,
                ep_type,
                ep_addr,
                max_packet_size,
                interval_ms,
            )?;
            Ok(EndpointOut {
                id: self.id,
                handle: ep,
            })
        }

        fn alloc_endpoint_in(
            &mut self,
            ep_type: EndpointType,
            ep_addr: Option<EndpointAddress>,
            max_packet_size: u16,
            interval_ms: u8,
        ) -> Result<Self::EndpointIn, EndpointAllocError> {
            let ep = Self::alloc_endpoint(
                &mut self.allocated_in,
                &mut self.in_info,
                Direction::In,
                ep_type,
                ep_addr,
                max_packet_size,
                interval_ms,
            )?;
            Ok(EndpointIn {
                id: self.id,
                handle: ep,
            })
        }

        fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
            with_shared_state(self.id, |state| {
                state.store_allocations(self.out_info, self.in_info);
            });
            (
                UsbBus { id: self.id },
                Control {
                    id: self.id,
                    max_packet_size: control_max_packet_size as usize,
                },
            )
        }
    }

    struct EndpointHandle {
        info: EndpointInfo,
    }

    /// Embassy USB bus handle.
    pub struct UsbBus {
        id: UsbId,
    }

    impl Bus for UsbBus {
        async fn enable(&mut self) {
            let result = with_shared_state(self.id, |state| state.start_device(self.id));
            if result.is_err() {
                fail_driver(self.id);
            } else {
                clear_driver_failure(self.id);
            }
        }

        async fn disable(&mut self) {
            with_shared_state(self.id, |state| {
                let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                device.disable();
                state.clear_cached_setup();
            });
            clear_driver_failure(self.id);
        }

        async fn poll(&mut self) -> Event {
            let rt = runtime(self.id);
            poll_fn(|cx| {
                rt.bus_waker.register(cx.waker());
                if let Some(event) = rt.take_event() {
                    if let Event::Reset = event {
                        let result = with_shared_state(self.id, |state| {
                            state.reset_after_bus_reset(self.id)
                        });
                        if result.is_err() {
                            fail_driver(self.id);
                        }
                    }
                    Poll::Ready(event)
                } else {
                    Poll::Pending
                }
            })
            .await
        }

        fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
            let result = with_shared_state(self.id, |state| -> Result<(), UsbError> {
                let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                if ep_addr.index() == 0 {
                    return Ok(());
                }
                let info = match ep_addr.direction() {
                    Direction::Out => state.out_info[ep_addr.index()],
                    Direction::In => state.in_info[ep_addr.index()],
                };
                let Some(info) = info else {
                    return Ok(());
                };
                device.configure_endpoint(ep_addr, info.ep_type, info.max_packet_size, enabled)?;
                if enabled && ep_addr.direction() == Direction::Out {
                    let buffer = &mut state.out_buffers[ep_addr.index()];
                    device.prime_out(ep_addr, buffer, info.max_packet_size as usize)?;
                }
                Ok(())
            });
            if result.is_err() {
                fail_driver(self.id);
            }
            runtime(self.id).control_waker.wake();
            runtime(self.id).data_in_waker.wake();
            runtime(self.id).data_out_waker.wake();
        }

        fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
            let result = with_shared_state(self.id, |state| {
                let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                device.set_stalled(ep_addr, stalled)
            });
            if result.is_err() {
                fail_driver(self.id);
            }
            runtime(self.id).control_waker.wake();
            runtime(self.id).data_in_waker.wake();
            runtime(self.id).data_out_waker.wake();
        }

        fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
            runtime(self.id).is_stalled(ep_addr)
        }

        async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
            Err(Unsupported)
        }
    }

    /// Embassy OUT endpoint handle.
    pub struct EndpointOut {
        id: UsbId,
        handle: EndpointHandle,
    }

    impl Endpoint for EndpointOut {
        fn info(&self) -> &EndpointInfo {
            &self.handle.info
        }

        async fn wait_enabled(&mut self) {
            let addr = self.handle.info.addr;
            let rt = runtime(self.id);
            poll_fn(|cx| {
                rt.data_out_waker.register(cx.waker());
                if rt.has_failed() || rt.is_enabled(addr) {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await
        }
    }

    impl EmbassyEndpointOut for EndpointOut {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
            let addr = self.handle.info.addr;
            let max_packet = self.handle.info.max_packet_size as usize;
            let rt = runtime(self.id);
            let reset_baseline = rt.reset_count();
            let mut scratch = [0u8; CONTROL_BUFFER_SIZE];
            poll_fn(|cx| {
                rt.data_out_waker.register(cx.waker());
                if !rt.is_enabled(addr) {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                if wait_for_reset(rt, reset_baseline).is_err() {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }

                let result = with_shared_state(self.id, |state| {
                    let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                    match device.take_transfer_complete(addr) {
                        Ok(Some(report)) => {
                            if let Err(err) =
                                device.finish_out(&mut state.out_buffers[addr.index()])
                            {
                                return Some(Err(map_usb_error(err)));
                            }
                            scratch[..report.actual_bytes].copy_from_slice(
                                &state.out_buffers[addr.index()].0[..report.actual_bytes],
                            );
                            let reprime = device.prime_out(
                                addr,
                                &mut state.out_buffers[addr.index()],
                                max_packet,
                            );
                            if let Err(err) = reprime {
                                Some(Err(map_usb_error(err)))
                            } else {
                                Some(Ok(report.actual_bytes))
                            }
                        }
                        Ok(None) => None,
                        Err(err) => Some(Err(map_usb_error(err))),
                    }
                });

                if let Some(result) = result {
                    Poll::Ready(match result {
                        Ok(len) => {
                            if len > buf.len() {
                                Err(EndpointError::BufferOverflow)
                            } else {
                                buf[..len].copy_from_slice(&scratch[..len]);
                                Ok(len)
                            }
                        }
                        Err(err) => Err(err),
                    })
                } else {
                    Poll::Pending
                }
            })
            .await
        }
    }

    /// Embassy IN endpoint handle.
    pub struct EndpointIn {
        id: UsbId,
        handle: EndpointHandle,
    }

    impl Endpoint for EndpointIn {
        fn info(&self) -> &EndpointInfo {
            &self.handle.info
        }

        async fn wait_enabled(&mut self) {
            let addr = self.handle.info.addr;
            let rt = runtime(self.id);
            poll_fn(|cx| {
                rt.data_in_waker.register(cx.waker());
                if rt.has_failed() || rt.is_enabled(addr) {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await
        }
    }

    impl EmbassyEndpointIn for EndpointIn {
        async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
            let addr = self.handle.info.addr;
            let rt = runtime(self.id);
            let reset_baseline = rt.reset_count();

            poll_fn(|cx| {
                rt.data_in_waker.register(cx.waker());
                if !rt.is_enabled(addr) {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                if wait_for_reset(rt, reset_baseline).is_err() {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }

                let result = with_shared_state(self.id, |state| {
                    let requested = state.resources.requested(addr);
                    let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                    if requested == 0 {
                        if buf.len() > state.in_buffers[addr.index()].0.len() {
                            return Some(Err(EndpointError::BufferOverflow));
                        }
                        state.in_buffers[addr.index()].0[..buf.len()].copy_from_slice(buf);
                        if let Err(err) =
                            device.prime_in(addr, &state.in_buffers[addr.index()], buf.len())
                        {
                            return Some(Err(map_usb_error(err)));
                        }
                        return None;
                    }
                    match device.take_transfer_complete(addr) {
                        Ok(Some(_)) => Some(Ok(())),
                        Ok(None) => None,
                        Err(err) => Some(Err(map_usb_error(err))),
                    }
                });

                if let Some(result) = result {
                    Poll::Ready(result)
                } else {
                    Poll::Pending
                }
            })
            .await
        }
    }

    /// Control-pipe handle used by `embassy-usb`.
    pub struct Control {
        id: UsbId,
        max_packet_size: usize,
    }

    impl Control {
        async fn wait_ep0_in_complete(&mut self, reset_baseline: u32) -> Result<(), EndpointError> {
            let rt = runtime(self.id);
            poll_fn(|cx| {
                rt.control_waker.register(cx.waker());
                if wait_for_reset(rt, reset_baseline).is_err() {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }

                let result = with_shared_state(self.id, |state| {
                    if maybe_capture_setup(state, self.id).unwrap_or(true) {
                        return Some(Err(EndpointError::Disabled));
                    }
                    let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                    match device
                        .take_transfer_complete(EndpointAddress::from_parts(0, Direction::In))
                    {
                        Ok(Some(_)) => Some(Ok(())),
                        Ok(None) => None,
                        Err(err) => Some(Err(map_usb_error(err))),
                    }
                });

                if let Some(result) = result {
                    Poll::Ready(result)
                } else {
                    Poll::Pending
                }
            })
            .await
        }

        async fn wait_ep0_out_complete(
            &mut self,
            buf: &mut [u8],
            reset_baseline: u32,
        ) -> Result<usize, EndpointError> {
            let rt = runtime(self.id);
            let mut scratch = [0u8; CONTROL_BUFFER_SIZE];
            poll_fn(|cx| {
                rt.control_waker.register(cx.waker());
                if wait_for_reset(rt, reset_baseline).is_err() {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }

                let result = with_shared_state(self.id, |state| {
                    if maybe_capture_setup(state, self.id).unwrap_or(true) {
                        return Some(Err(EndpointError::Disabled));
                    }
                    let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                    match device.take_ep0_out_data(&mut scratch) {
                        Ok(Some(len)) => Some(Ok(len)),
                        Ok(None) => None,
                        Err(err) => Some(Err(map_usb_error(err))),
                    }
                });

                if let Some(result) = result {
                    Poll::Ready(match result {
                        Ok(len) => {
                            if len > buf.len() {
                                Err(EndpointError::BufferOverflow)
                            } else {
                                buf[..len].copy_from_slice(&scratch[..len]);
                                Ok(len)
                            }
                        }
                        Err(err) => Err(err),
                    })
                } else {
                    Poll::Pending
                }
            })
            .await
        }
    }

    impl ControlPipe for Control {
        fn max_packet_size(&self) -> usize {
            self.max_packet_size
        }

        async fn setup(&mut self) -> [u8; 8] {
            let rt = runtime(self.id);
            poll_fn(|cx| {
                rt.control_waker.register(cx.waker());
                let packet = with_shared_state(self.id, |state| {
                    if maybe_capture_setup(state, self.id).ok()? {
                        state.setup_packet.take()
                    } else {
                        None
                    }
                });
                if let Some(packet) = packet {
                    Poll::Ready(packet)
                } else {
                    Poll::Pending
                }
            })
            .await
        }

        async fn data_out(
            &mut self,
            buf: &mut [u8],
            _first: bool,
            _last: bool,
        ) -> Result<usize, EndpointError> {
            let reset_baseline = runtime(self.id).reset_count();
            with_shared_state(self.id, |state| {
                let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                device.prime_ep0_out_data(core::cmp::min(buf.len(), self.max_packet_size))
            })
            .map_err(map_usb_error)?;
            self.wait_ep0_out_complete(buf, reset_baseline).await
        }

        async fn data_in(
            &mut self,
            data: &[u8],
            _first: bool,
            last: bool,
        ) -> Result<(), EndpointError> {
            let reset_baseline = runtime(self.id).reset_count();
            with_shared_state(self.id, |state| {
                let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                device.prime_ep0_in(data, core::cmp::min(data.len(), self.max_packet_size))
            })
            .map_err(map_usb_error)?;
            self.wait_ep0_in_complete(reset_baseline).await?;

            if last {
                with_shared_state(self.id, |state| {
                    let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                    device.prime_ep0_out_status()
                })
                .map_err(map_usb_error)?;
                let mut status_buf = [];
                self.wait_ep0_out_complete(&mut status_buf, reset_baseline)
                    .await?;
            }
            Ok(())
        }

        async fn accept(&mut self) {
            let reset_baseline = runtime(self.id).reset_count();
            let result = with_shared_state(self.id, |state| -> Result<(), UsbError> {
                let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                device.set_stalled(EndpointAddress::from_parts(0, Direction::Out), false)?;
                device.set_stalled(EndpointAddress::from_parts(0, Direction::In), false)?;
                device.prime_ep0_in_status()?;
                Ok(())
            });
            if result.is_err() || self.wait_ep0_in_complete(reset_baseline).await.is_err() {
                fail_driver(self.id);
            }
        }

        async fn reject(&mut self) {
            let result = with_shared_state(self.id, |state| {
                let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                device.stall_ep0()
            });
            if result.is_err() {
                fail_driver(self.id);
            }
        }

        async fn accept_set_address(&mut self, addr: u8) {
            let reset_baseline = runtime(self.id).reset_count();
            let result = with_shared_state(self.id, |state| -> Result<(), UsbError> {
                let mut device = fresh_controller(self.id).into_device(&mut state.resources);
                device.set_stalled(ep0_addr(Direction::Out), false)?;
                device.set_stalled(ep0_addr(Direction::In), false)?;
                device.arm_address_after_status(addr);
                device.prime_ep0_in_status()?;
                Ok(())
            });
            if result.is_err() || self.wait_ep0_in_complete(reset_baseline).await.is_err() {
                fail_driver(self.id);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn runtime_events_prioritize_reset_then_suspend_then_resume() {
        let runtime = ControllerRuntime::new();
        runtime.push_event(EVENT_RESUME);
        runtime.push_event(EVENT_SUSPEND);
        runtime.push_event(EVENT_RESET);

        assert!(matches!(runtime.take_event(), Some(Event::Reset)));
        assert!(matches!(runtime.take_event(), Some(Event::Suspend)));
        assert!(matches!(runtime.take_event(), Some(Event::Resume)));
        assert!(runtime.take_event().is_none());
    }

    #[test]
    fn port_change_only_reports_resume_when_previously_suspended() {
        assert_eq!(port_change_event_bits(false, true), None);
        assert_eq!(port_change_event_bits(false, false), None);
        assert_eq!(port_change_event_bits(true, false), None);
        assert_eq!(port_change_event_bits(true, true), Some(EVENT_RESUME));
    }

    #[test]
    fn staged_device_address_sets_address_and_advance_bit() {
        let staged = staged_device_address(42);
        assert_eq!(staged.usb_address().value(), 42);
        assert!(staged.address_advance());
    }

    #[test]
    fn otgsc_power_decode_uses_real_session_and_vbus_bits() {
        assert!(!power_present_from_otgsc(OtgSc::new_with_raw_value(0)));
        assert!(power_present_from_otgsc(OtgSc::new_with_raw_value(1 << 11)));
        assert!(power_present_from_otgsc(OtgSc::new_with_raw_value(1 << 10)));
        assert!(power_present_from_otgsc(OtgSc::new_with_raw_value(1 << 9)));
        assert!(!power_present_from_otgsc(OtgSc::new_with_raw_value(
            1 << 12
        )));
    }

    #[test]
    fn power_transition_emits_detected_and_removed_events() {
        assert_eq!(power_event_bits_for_transition(false, false), None);
        assert_eq!(
            power_event_bits_for_transition(false, true),
            Some(EVENT_POWER_DETECTED)
        );
        assert_eq!(power_event_bits_for_transition(true, true), None);
        assert_eq!(
            power_event_bits_for_transition(true, false),
            Some(EVENT_POWER_REMOVED)
        );
    }

    #[test]
    fn clear_endpoint_state_clears_enable_stall_and_completion_bits() {
        let runtime = ControllerRuntime::new();
        let ep1_out = EndpointAddress::from_parts(1, Direction::Out);
        let ep1_in = EndpointAddress::from_parts(1, Direction::In);

        runtime.set_enabled(ep1_out, true);
        runtime.set_enabled(ep1_in, true);
        runtime.set_stalled(ep1_out, true);
        runtime.set_stalled(ep1_in, true);
        runtime.push_completed(Direction::Out, endpoint_mask(ep1_out));
        runtime.push_completed(Direction::In, endpoint_mask(ep1_in));

        runtime.clear_endpoint_state();

        assert!(!runtime.is_enabled(ep1_out));
        assert!(!runtime.is_enabled(ep1_in));
        assert!(!runtime.is_stalled(ep1_out));
        assert!(!runtime.is_stalled(ep1_in));
        assert!(!runtime.take_completed(ep1_out));
        assert!(!runtime.take_completed(ep1_in));
    }

    #[test]
    fn clear_ep0_transfer_state_resets_bookkeeping_and_descriptors() {
        let mut resources = UsbResources::new();
        let ep0_out = ep0_addr(Direction::Out);
        let ep0_in = ep0_addr(Direction::In);

        resources.set_requested(ep0_out, 17);
        resources.set_requested(ep0_in, 9);
        resources.dtd_mut(ep0_out).configure(0x1000, 17, true);
        resources.dtd_mut(ep0_in).configure(0x2000, 9, true);
        resources
            .queue_heads
            .endpoint(0, Direction::Out)
            .attach_dtd(resources.dtd(ep0_out));
        resources
            .queue_heads
            .endpoint(0, Direction::In)
            .attach_dtd(resources.dtd(ep0_in));
        resources
            .queue_heads
            .endpoint(0, Direction::Out)
            .token
            .set(0x1234);
        resources
            .queue_heads
            .endpoint(0, Direction::In)
            .token
            .set(0x5678);

        resources.clear_ep0_transfer_state();

        assert_eq!(resources.requested(ep0_out), 0);
        assert_eq!(resources.requested(ep0_in), 0);
        assert_eq!(resources.dtd(ep0_out).token.get(), 0);
        assert_eq!(resources.dtd(ep0_in).token.get(), 0);
        assert_eq!(resources.dtd(ep0_out).next_dtd.get(), DTD_NEXT_TERMINATE);
        assert_eq!(resources.dtd(ep0_in).next_dtd.get(), DTD_NEXT_TERMINATE);
        assert_eq!(
            resources
                .queue_heads
                .endpoint(0, Direction::Out)
                .next_dtd
                .get(),
            DTD_NEXT_TERMINATE
        );
        assert_eq!(
            resources
                .queue_heads
                .endpoint(0, Direction::In)
                .next_dtd
                .get(),
            DTD_NEXT_TERMINATE
        );
        assert_eq!(
            resources
                .queue_heads
                .endpoint(0, Direction::Out)
                .token
                .get(),
            0
        );
        assert_eq!(
            resources.queue_heads.endpoint(0, Direction::In).token.get(),
            0
        );
    }

    #[test]
    fn reset_state_clears_all_endpoint_bookkeeping() {
        let mut resources = UsbResources::new();
        let ep1_out = EndpointAddress::from_parts(1, Direction::Out);
        let ep1_in = EndpointAddress::from_parts(1, Direction::In);

        resources.set_requested(ep1_out, 23);
        resources.set_requested(ep1_in, 11);
        resources.dtd_mut(ep1_out).configure(0x3000, 23, true);
        resources.dtd_mut(ep1_in).configure(0x4000, 11, true);
        resources
            .queue_heads
            .endpoint(1, Direction::Out)
            .attach_dtd(resources.dtd(ep1_out));
        resources
            .queue_heads
            .endpoint(1, Direction::In)
            .attach_dtd(resources.dtd(ep1_in));

        resources.reset_state();

        assert_eq!(resources.requested(ep1_out), 0);
        assert_eq!(resources.requested(ep1_in), 0);
        assert_eq!(resources.dtd(ep1_out).token.get(), 0);
        assert_eq!(resources.dtd(ep1_in).token.get(), 0);
        assert_eq!(
            resources
                .queue_heads
                .endpoint(1, Direction::Out)
                .next_dtd
                .get(),
            DTD_NEXT_TERMINATE
        );
        assert_eq!(
            resources
                .queue_heads
                .endpoint(1, Direction::In)
                .next_dtd
                .get(),
            DTD_NEXT_TERMINATE
        );
    }
}
