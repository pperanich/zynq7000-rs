//! Embassy USB driver facade for the Zynq-7000 PS USB controller.

use core::{cell::RefCell, future::poll_fn, marker::PhantomData, task::Poll};

use critical_section::Mutex;
use embassy_hal_internal::Peri;
use embassy_usb_driver::{
    Bus, ControlPipe, Direction, Endpoint, EndpointAddress, EndpointAllocError, EndpointError,
    EndpointIn as EmbassyEndpointIn, EndpointInfo, EndpointOut as EmbassyEndpointOut, EndpointType,
    Event, Unsupported,
};

use super::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ControlPhase {
    Idle,
    SetupPending([u8; 8]),
    DataOut { last: bool },
    DataIn { last: bool },
    AwaitingAccept,
    StatusOut,
    StatusIn,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ControlPoll<T> {
    Pending,
    Ready(T),
    Interrupted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum OutCompletion {
    Data(usize),
    AwaitingAccept(usize),
    StatusComplete,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum InCompletion {
    DataComplete,
    NeedsStatusOut,
    StatusComplete,
}

struct ControlState {
    phase: ControlPhase,
}

impl ControlState {
    const fn new() -> Self {
        Self {
            phase: ControlPhase::Idle,
        }
    }

    fn record_setup_packet(&mut self, packet: [u8; 8]) {
        self.phase = ControlPhase::SetupPending(packet);
    }

    fn clear(&mut self) {
        self.phase = ControlPhase::Idle;
    }

    fn capture_setup_packet(&mut self, device: &mut UsbDevice<'_>) -> Result<bool, UsbError> {
        if let Some(packet) = device.read_setup_packet()? {
            device.abort_ep0_state()?;
            self.record_setup_packet(packet);
            return Ok(true);
        }
        Ok(matches!(self.phase, ControlPhase::SetupPending(_)))
    }

    fn take_setup_packet(&mut self) -> Option<[u8; 8]> {
        match self.phase {
            ControlPhase::SetupPending(packet) => {
                self.phase = ControlPhase::Idle;
                Some(packet)
            }
            _ => None,
        }
    }

    fn start_data_out(&mut self, last: bool) {
        self.phase = ControlPhase::DataOut { last };
    }

    fn finish_out(&mut self, len: usize) -> OutCompletion {
        match self.phase {
            ControlPhase::DataOut { last: false } => {
                self.phase = ControlPhase::Idle;
                OutCompletion::Data(len)
            }
            ControlPhase::DataOut { last: true } => {
                self.phase = ControlPhase::AwaitingAccept;
                OutCompletion::AwaitingAccept(len)
            }
            ControlPhase::StatusOut => {
                self.phase = ControlPhase::Idle;
                OutCompletion::StatusComplete
            }
            _ => {
                self.phase = ControlPhase::Idle;
                OutCompletion::Data(len)
            }
        }
    }

    fn prime_out_data(
        &mut self,
        device: &mut UsbDevice<'_>,
        expected_len: usize,
        last: bool,
    ) -> Result<usize, UsbError> {
        self.start_data_out(last);
        device.prime_ep0_out_data(expected_len)
    }

    fn take_out_data(
        &mut self,
        device: &mut UsbDevice<'_>,
        dst: &mut [u8],
    ) -> Result<ControlPoll<OutCompletion>, UsbError> {
        if self.capture_setup_packet(device)? {
            return Ok(ControlPoll::Interrupted);
        }
        match device.take_ep0_out_data(dst)? {
            Some(len) => Ok(ControlPoll::Ready(self.finish_out(len))),
            None => Ok(ControlPoll::Pending),
        }
    }

    fn start_data_in(&mut self, last: bool) {
        self.phase = ControlPhase::DataIn { last };
    }

    fn finish_in(&mut self) -> InCompletion {
        match self.phase {
            ControlPhase::DataIn { last: false } => {
                self.phase = ControlPhase::Idle;
                InCompletion::DataComplete
            }
            ControlPhase::DataIn { last: true } => {
                self.phase = ControlPhase::Idle;
                InCompletion::NeedsStatusOut
            }
            ControlPhase::StatusIn => {
                self.phase = ControlPhase::Idle;
                InCompletion::StatusComplete
            }
            _ => {
                self.phase = ControlPhase::Idle;
                InCompletion::DataComplete
            }
        }
    }

    fn prime_in_data(
        &mut self,
        device: &mut UsbDevice<'_>,
        data: &[u8],
        requested_len: usize,
        last: bool,
    ) -> Result<usize, UsbError> {
        self.start_data_in(last);
        device.prime_ep0_in(data, requested_len)
    }

    fn take_in_complete(
        &mut self,
        device: &mut UsbDevice<'_>,
    ) -> Result<ControlPoll<InCompletion>, UsbError> {
        if self.capture_setup_packet(device)? {
            return Ok(ControlPoll::Interrupted);
        }
        match device.take_transfer_complete(ep0_addr(Direction::In))? {
            Some(_) => Ok(ControlPoll::Ready(self.finish_in())),
            None => Ok(ControlPoll::Pending),
        }
    }

    fn prime_out_status(&mut self, device: &mut UsbDevice<'_>) -> Result<(), UsbError> {
        self.phase = ControlPhase::StatusOut;
        device.prime_ep0_out_status()
    }

    fn stall(&mut self, device: &mut UsbDevice<'_>) -> Result<(), UsbError> {
        self.clear();
        device.stall_ep0()
    }

    fn accept(&mut self, device: &mut UsbDevice<'_>) -> Result<(), UsbError> {
        device.set_stalled(ep0_addr(Direction::Out), false)?;
        device.set_stalled(ep0_addr(Direction::In), false)?;
        self.phase = ControlPhase::StatusIn;
        device.prime_ep0_in_status()
    }

    fn accept_set_address(&mut self, device: &mut UsbDevice<'_>, addr: u8) -> Result<(), UsbError> {
        device.set_stalled(ep0_addr(Direction::Out), false)?;
        device.set_stalled(ep0_addr(Direction::In), false)?;
        device.arm_address_after_status(addr);
        self.phase = ControlPhase::StatusIn;
        device.prime_ep0_in_status()
    }
}

struct SharedState {
    resources: UsbResources,
    out_info: [Option<EndpointInfo>; MAX_ENDPOINTS],
    in_info: [Option<EndpointInfo>; MAX_ENDPOINTS],
    out_buffers: [AlignedUsbBuffer<DATA_BUFFER_SIZE>; MAX_ENDPOINTS],
    in_buffers: [AlignedUsbBuffer<DATA_BUFFER_SIZE>; MAX_ENDPOINTS],
    pending_out_len: [Option<usize>; MAX_ENDPOINTS],
    control_out_buffer: [u8; EP0_BUFFER_SIZE],
    pending_control_out: Option<(OutCompletion, usize)>,
    control: ControlState,
    control_max_packet_size: u16,
}

impl SharedState {
    const fn new() -> Self {
        Self {
            resources: UsbResources::new(),
            out_info: [None; MAX_ENDPOINTS],
            in_info: [None; MAX_ENDPOINTS],
            out_buffers: [const { AlignedUsbBuffer::new() }; MAX_ENDPOINTS],
            in_buffers: [const { AlignedUsbBuffer::new() }; MAX_ENDPOINTS],
            pending_out_len: [None; MAX_ENDPOINTS],
            control_out_buffer: [0; EP0_BUFFER_SIZE],
            pending_control_out: None,
            control: ControlState::new(),
            control_max_packet_size: 64,
        }
    }

    fn store_allocations(
        &mut self,
        out_info: [Option<EndpointInfo>; MAX_ENDPOINTS],
        in_info: [Option<EndpointInfo>; MAX_ENDPOINTS],
        control_max_packet_size: u16,
    ) {
        self.resources = UsbResources::new();
        self.out_info = out_info;
        self.in_info = in_info;
        self.pending_out_len = [None; MAX_ENDPOINTS];
        self.control_max_packet_size = control_max_packet_size;
        self.clear_control_state();
    }

    fn clear_control_state(&mut self) {
        self.pending_control_out = None;
        self.control.clear();
    }

    fn control_take_setup_packet(&mut self, id: UsbId) -> Result<Option<[u8; 8]>, UsbError> {
        let control = &mut self.control;
        let resources = &mut self.resources;
        let mut device = fresh_controller(id).into_device(resources);
        if control.capture_setup_packet(&mut device)? {
            Ok(control.take_setup_packet())
        } else {
            Ok(None)
        }
    }

    fn with_device<R>(&mut self, id: UsbId, f: impl FnOnce(&mut UsbDevice<'_>) -> R) -> R {
        let mut device = fresh_controller(id).into_device(&mut self.resources);
        f(&mut device)
    }

    fn start_device(&mut self, id: UsbId) -> Result<(), UsbError> {
        self.clear_control_state();
        let control_max_packet_size = self.control_max_packet_size;
        self.with_device(id, |device| {
            device.start_device_mode(control_max_packet_size)
        })
    }

    fn reset_after_bus_reset(&mut self, id: UsbId) -> Result<(), UsbError> {
        self.clear_control_state();
        let control_max_packet_size = self.control_max_packet_size;
        self.with_device(id, |device| device.on_bus_reset(control_max_packet_size))
    }

    fn control_prime_out_data(
        &mut self,
        id: UsbId,
        expected_len: usize,
        last: bool,
    ) -> Result<usize, UsbError> {
        let control = &mut self.control;
        let resources = &mut self.resources;
        let mut device = fresh_controller(id).into_device(resources);
        control.prime_out_data(&mut device, expected_len, last)
    }

    fn control_take_out_data(
        &mut self,
        id: UsbId,
        dst: &mut [u8],
    ) -> Result<ControlPoll<OutCompletion>, UsbError> {
        let control = &mut self.control;
        let resources = &mut self.resources;
        let mut device = fresh_controller(id).into_device(resources);

        if control.capture_setup_packet(&mut device)? {
            self.pending_control_out = None;
            return Ok(ControlPoll::Interrupted);
        }

        if let Some((done, len)) = self.pending_control_out {
            if len > dst.len() {
                return Err(UsbError::BufferTooSmall {
                    requested: len,
                    buffer_len: dst.len(),
                });
            }
            dst[..len].copy_from_slice(&self.control_out_buffer[..len]);
            self.pending_control_out = None;
            return Ok(ControlPoll::Ready(done));
        }

        match control.take_out_data(&mut device, &mut self.control_out_buffer)? {
            ControlPoll::Ready(done) => {
                let len = match done {
                    OutCompletion::Data(len) | OutCompletion::AwaitingAccept(len) => len,
                    OutCompletion::StatusComplete => 0,
                };
                if len > dst.len() {
                    self.pending_control_out = Some((done, len));
                    return Err(UsbError::BufferTooSmall {
                        requested: len,
                        buffer_len: dst.len(),
                    });
                }
                dst[..len].copy_from_slice(&self.control_out_buffer[..len]);
                Ok(ControlPoll::Ready(done))
            }
            other => Ok(other),
        }
    }

    fn control_prime_in_data(
        &mut self,
        id: UsbId,
        data: &[u8],
        requested_len: usize,
        last: bool,
    ) -> Result<usize, UsbError> {
        let control = &mut self.control;
        let resources = &mut self.resources;
        let mut device = fresh_controller(id).into_device(resources);
        control.prime_in_data(&mut device, data, requested_len, last)
    }

    fn control_take_in_complete(
        &mut self,
        id: UsbId,
    ) -> Result<ControlPoll<InCompletion>, UsbError> {
        let control = &mut self.control;
        let resources = &mut self.resources;
        let mut device = fresh_controller(id).into_device(resources);
        control.take_in_complete(&mut device)
    }

    fn control_prime_out_status(&mut self, id: UsbId) -> Result<(), UsbError> {
        let control = &mut self.control;
        let resources = &mut self.resources;
        let mut device = fresh_controller(id).into_device(resources);
        control.prime_out_status(&mut device)
    }

    fn control_stall(&mut self, id: UsbId) -> Result<(), UsbError> {
        let control = &mut self.control;
        let resources = &mut self.resources;
        let mut device = fresh_controller(id).into_device(resources);
        control.stall(&mut device)
    }

    fn control_accept(&mut self, id: UsbId) -> Result<(), UsbError> {
        let control = &mut self.control;
        let resources = &mut self.resources;
        let mut device = fresh_controller(id).into_device(resources);
        control.accept(&mut device)
    }

    fn control_accept_set_address(&mut self, id: UsbId, addr: u8) -> Result<(), UsbError> {
        let control = &mut self.control;
        let resources = &mut self.resources;
        let mut device = fresh_controller(id).into_device(resources);
        control.accept_set_address(&mut device, addr)
    }

    fn endpoint_set_enabled(
        &mut self,
        id: UsbId,
        ep_addr: EndpointAddress,
        enabled: bool,
    ) -> Result<(), UsbError> {
        if ep_addr.index() == 0 {
            return Ok(());
        }
        let info = match ep_addr.direction() {
            Direction::Out => self.out_info[ep_addr.index()],
            Direction::In => self.in_info[ep_addr.index()],
        };
        let Some(info) = info else {
            return Ok(());
        };
        let resources = &mut self.resources;
        let out_buffers = &mut self.out_buffers;
        let mut device = fresh_controller(id).into_device(resources);
        {
            device.configure_endpoint(ep_addr, info.ep_type, info.max_packet_size, enabled)?;
            if enabled && ep_addr.direction() == Direction::Out {
                let buffer = &mut out_buffers[ep_addr.index()];
                device.prime_out(ep_addr, buffer, info.max_packet_size as usize)?;
            }
            Ok(())
        }
    }

    fn endpoint_set_stalled(
        &mut self,
        id: UsbId,
        ep_addr: EndpointAddress,
        stalled: bool,
    ) -> Result<(), UsbError> {
        self.with_device(id, |device| device.set_stalled(ep_addr, stalled))
    }

    fn endpoint_take_out_data(
        &mut self,
        id: UsbId,
        addr: EndpointAddress,
        scratch: &mut [u8; DATA_BUFFER_SIZE],
        max_packet: usize,
        available_len: usize,
    ) -> Result<Option<usize>, EndpointError> {
        if let Some(len) = self.pending_out_len[addr.index()] {
            if len > available_len {
                return Err(EndpointError::BufferOverflow);
            }
            scratch[..len].copy_from_slice(&self.out_buffers[addr.index()].0[..len]);
            self.pending_out_len[addr.index()] = None;
            let resources = &mut self.resources;
            let out_buffers = &mut self.out_buffers;
            let mut device = fresh_controller(id).into_device(resources);
            device
                .prime_out(addr, &mut out_buffers[addr.index()], max_packet)
                .map_err(map_usb_error)?;
            return Ok(Some(len));
        }

        match self.with_device(id, |device| device.take_transfer_complete(addr)) {
            Ok(Some(report)) => {
                if report.actual_bytes > available_len {
                    let resources = &mut self.resources;
                    let out_buffers = &mut self.out_buffers;
                    let mut device = fresh_controller(id).into_device(resources);
                    device
                        .finish_out(&mut out_buffers[addr.index()])
                        .map_err(map_usb_error)?;
                    self.pending_out_len[addr.index()] = Some(report.actual_bytes);
                    return Err(EndpointError::BufferOverflow);
                }
                let resources = &mut self.resources;
                let out_buffers = &mut self.out_buffers;
                let mut device = fresh_controller(id).into_device(resources);
                device
                    .finish_out(&mut out_buffers[addr.index()])
                    .map_err(map_usb_error)?;
                scratch[..report.actual_bytes]
                    .copy_from_slice(&out_buffers[addr.index()].0[..report.actual_bytes]);
                device
                    .prime_out(addr, &mut out_buffers[addr.index()], max_packet)
                    .map_err(map_usb_error)?;
                Ok(Some(report.actual_bytes))
            }
            Ok(None) => Ok(None),
            Err(err) => Err(map_usb_error(err)),
        }
    }

    fn endpoint_write_in(
        &mut self,
        id: UsbId,
        addr: EndpointAddress,
        buf: &[u8],
    ) -> Result<(), EndpointError> {
        let resources = &mut self.resources;
        let in_buffers = &mut self.in_buffers;
        if buf.len() > in_buffers[addr.index()].0.len() {
            return Err(EndpointError::BufferOverflow);
        }
        in_buffers[addr.index()].0[..buf.len()].copy_from_slice(buf);
        let mut device = fresh_controller(id).into_device(resources);
        device
            .prime_in(addr, &in_buffers[addr.index()], buf.len())
            .map_err(map_usb_error)
    }

    fn endpoint_take_in_complete(
        &mut self,
        id: UsbId,
        addr: EndpointAddress,
    ) -> Result<Option<()>, EndpointError> {
        match self.with_device(id, |device| device.take_transfer_complete(addr)) {
            Ok(Some(_)) => Ok(Some(())),
            Ok(None) => Ok(None),
            Err(err) => Err(map_usb_error(err)),
        }
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
    rt.clear_events();
    rt.bus_waker.wake();
    rt.control_waker.wake();
    rt.wake_all_endpoints();
}

fn clear_driver_failure(id: UsbId) {
    runtime(id).set_failed(false);
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
        super::on_interrupt(T::id());
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
    /// Create a new Embassy USB driver from a token-backed controller instance.
    pub fn new<T: super::Instance>(
        _usb: Peri<'d, T>,
        irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        let id = T::id();
        bind_usb_interrupt::<T, _>(irq);
        Self {
            id,
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
        if ep_type == EndpointType::Isochronous || max_packet_size as usize > DATA_BUFFER_SIZE {
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

fn bind_usb_interrupt<T, B>(_binding: B)
where
    T: super::Instance,
    B: crate::interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
{
    B::register();
    <T::Interrupt as crate::interrupt::typelevel::Interrupt>::unpend();
    <T::Interrupt as crate::interrupt::typelevel::Interrupt>::enable();
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
            state.store_allocations(self.out_info, self.in_info, control_max_packet_size);
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
            state.with_device(self.id, |device| device.disable());
            state.clear_control_state();
        });
        clear_driver_failure(self.id);
    }

    async fn poll(&mut self) -> Event {
        let rt = runtime(self.id);
        poll_fn(|cx| {
            rt.bus_waker.register(cx.waker());
            if rt.has_failed() {
                clear_driver_failure(self.id);
                rt.control_waker.wake();
                rt.wake_all_endpoints();
                return Poll::Ready(Event::PowerRemoved);
            }
            if let Some(event) = rt.take_event() {
                if let Event::Reset = event {
                    let result =
                        with_shared_state(self.id, |state| state.reset_after_bus_reset(self.id));
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
        let result = with_shared_state(self.id, |state| {
            state.endpoint_set_enabled(self.id, ep_addr, enabled)
        });
        if result.is_err() {
            fail_driver(self.id);
        }
        runtime(self.id).control_waker.wake();
        runtime(self.id).wake_endpoint(ep_addr);
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let result = with_shared_state(self.id, |state| {
            state.endpoint_set_stalled(self.id, ep_addr, stalled)
        });
        if result.is_err() {
            fail_driver(self.id);
        }
        runtime(self.id).control_waker.wake();
        runtime(self.id).wake_endpoint(ep_addr);
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
            rt.endpoint_waker(addr).register(cx.waker());
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
        let mut scratch = [0u8; DATA_BUFFER_SIZE];
        poll_fn(|cx| {
            rt.endpoint_waker(addr).register(cx.waker());
            if !rt.is_enabled(addr) {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            if wait_for_reset(rt, reset_baseline).is_err() {
                return Poll::Ready(Err(EndpointError::Disabled));
            }

            let result = with_shared_state(self.id, |state| {
                match state.endpoint_take_out_data(
                    self.id,
                    addr,
                    &mut scratch,
                    max_packet,
                    buf.len(),
                ) {
                    Ok(Some(len)) => Some(Ok(len)),
                    Ok(None) => None,
                    Err(err) => Some(Err(err)),
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
            rt.endpoint_waker(addr).register(cx.waker());
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
            rt.endpoint_waker(addr).register(cx.waker());
            if !rt.is_enabled(addr) {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            if wait_for_reset(rt, reset_baseline).is_err() {
                return Poll::Ready(Err(EndpointError::Disabled));
            }

            let result = with_shared_state(self.id, |state| {
                let requested = state.resources.requested(addr);
                if requested == 0 {
                    if let Err(err) = state.endpoint_write_in(self.id, addr, buf) {
                        return Some(Err(err));
                    }
                    return None;
                }
                match state.endpoint_take_in_complete(self.id, addr) {
                    Ok(Some(())) => Some(Ok(())),
                    Ok(None) => None,
                    Err(err) => Some(Err(err)),
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

fn status_in_poll_result(poll: ControlPoll<InCompletion>) -> Option<Result<(), EndpointError>> {
    match poll {
        ControlPoll::Ready(_) => Some(Ok(())),
        ControlPoll::Interrupted => Some(Err(EndpointError::Disabled)),
        ControlPoll::Pending => None,
    }
}

impl Control {
    async fn wait_ep0_in_complete(
        &mut self,
        reset_baseline: u32,
    ) -> Result<InCompletion, EndpointError> {
        let rt = runtime(self.id);
        poll_fn(|cx| {
            rt.control_waker.register(cx.waker());
            if wait_for_reset(rt, reset_baseline).is_err() {
                return Poll::Ready(Err(EndpointError::Disabled));
            }

            let result = with_shared_state(self.id, |state| {
                match state.control_take_in_complete(self.id) {
                    Ok(ControlPoll::Ready(done)) => Some(Ok(done)),
                    Ok(ControlPoll::Pending) => None,
                    Ok(ControlPoll::Interrupted) => Some(Err(EndpointError::Disabled)),
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

    async fn wait_ep0_status_in_complete(
        &mut self,
        reset_baseline: u32,
    ) -> Result<(), EndpointError> {
        let rt = runtime(self.id);
        poll_fn(|cx| {
            rt.control_waker.register(cx.waker());
            if wait_for_reset(rt, reset_baseline).is_err() {
                return Poll::Ready(Err(EndpointError::Disabled));
            }

            let result = with_shared_state(self.id, |state| {
                match state.control_take_in_complete(self.id) {
                    Ok(poll) => status_in_poll_result(poll),
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
    ) -> Result<OutCompletion, EndpointError> {
        let rt = runtime(self.id);
        let mut scratch = [0u8; EP0_BUFFER_SIZE];
        poll_fn(|cx| {
            rt.control_waker.register(cx.waker());
            if wait_for_reset(rt, reset_baseline).is_err() {
                return Poll::Ready(Err(EndpointError::Disabled));
            }

            let result = with_shared_state(self.id, |state| {
                match state.control_take_out_data(self.id, &mut scratch) {
                    Ok(ControlPoll::Ready(done)) => Some(Ok(done)),
                    Ok(ControlPoll::Pending) => None,
                    Ok(ControlPoll::Interrupted) => Some(Err(EndpointError::Disabled)),
                    Err(err) => Some(Err(map_usb_error(err))),
                }
            });

            if let Some(result) = result {
                Poll::Ready(match result {
                    Ok(done) => match done {
                        OutCompletion::Data(len) | OutCompletion::AwaitingAccept(len) => {
                            if len > buf.len() {
                                Err(EndpointError::BufferOverflow)
                            } else {
                                buf[..len].copy_from_slice(&scratch[..len]);
                                Ok(done)
                            }
                        }
                        OutCompletion::StatusComplete => Ok(done),
                    },
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
        let mut reset_baseline = rt.reset_count();
        poll_fn(|cx| {
            rt.control_waker.register(cx.waker());
            if wait_for_reset(rt, reset_baseline).is_err() {
                let _ = with_shared_state(self.id, |state| {
                    state.clear_control_state();
                });
                reset_baseline = rt.reset_count();
                return Poll::Pending;
            }

            match with_shared_state(self.id, |state| state.control_take_setup_packet(self.id)) {
                Ok(Some(packet)) => Poll::Ready(packet),
                Ok(None) => {
                    if rt.reset_count() != reset_baseline {
                        let _ = with_shared_state(self.id, |state| {
                            state.clear_control_state();
                        });
                        reset_baseline = rt.reset_count();
                        Poll::Pending
                    } else {
                        Poll::Pending
                    }
                }
                Err(_) => {
                    fail_driver(self.id);
                    let _ = with_shared_state(self.id, |state| {
                        state.clear_control_state();
                    });
                    Poll::Pending
                }
            }
        })
        .await
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        _first: bool,
        last: bool,
    ) -> Result<usize, EndpointError> {
        let reset_baseline = runtime(self.id).reset_count();
        let expected = self.max_packet_size;
        with_shared_state(self.id, |state| {
            state.control_prime_out_data(self.id, expected, last)
        })
        .map_err(map_usb_error)?;

        match self.wait_ep0_out_complete(buf, reset_baseline).await? {
            OutCompletion::Data(len) | OutCompletion::AwaitingAccept(len) => Ok(len),
            OutCompletion::StatusComplete => Ok(0),
        }
    }

    async fn data_in(
        &mut self,
        data: &[u8],
        _first: bool,
        last: bool,
    ) -> Result<(), EndpointError> {
        let reset_baseline = runtime(self.id).reset_count();
        let packet_len = core::cmp::min(data.len(), self.max_packet_size);
        with_shared_state(self.id, |state| {
            state.control_prime_in_data(self.id, &data[..packet_len], packet_len, last)
        })
        .map_err(map_usb_error)?;

        match self.wait_ep0_in_complete(reset_baseline).await? {
            InCompletion::DataComplete | InCompletion::StatusComplete => Ok(()),
            InCompletion::NeedsStatusOut => {
                with_shared_state(self.id, |state| state.control_prime_out_status(self.id))
                    .map_err(map_usb_error)?;
                let mut status_buf = [];
                let _ = self
                    .wait_ep0_out_complete(&mut status_buf, reset_baseline)
                    .await?;
                Ok(())
            }
        }
    }

    async fn accept(&mut self) {
        let reset_baseline = runtime(self.id).reset_count();
        let result = with_shared_state(self.id, |state| state.control_accept(self.id));
        if result.is_err()
            || self
                .wait_ep0_status_in_complete(reset_baseline)
                .await
                .is_err()
        {
            fail_driver(self.id);
        }
    }

    async fn reject(&mut self) {
        let result = with_shared_state(self.id, |state| state.control_stall(self.id));
        if result.is_err() {
            fail_driver(self.id);
        }
    }

    async fn accept_set_address(&mut self, addr: u8) {
        let reset_baseline = runtime(self.id).reset_count();
        let result = with_shared_state(self.id, |state| {
            state.control_accept_set_address(self.id, addr)
        });
        if result.is_err()
            || self
                .wait_ep0_status_in_complete(reset_baseline)
                .await
                .is_err()
        {
            fail_driver(self.id);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        ControlPhase, ControlPoll, ControlState, EndpointError, InCompletion, OutCompletion,
    };

    #[test]
    fn take_setup_packet_clears_pending_setup() {
        let mut state = ControlState {
            phase: ControlPhase::SetupPending([1, 2, 3, 4, 5, 6, 7, 8]),
        };

        assert_eq!(state.take_setup_packet(), Some([1, 2, 3, 4, 5, 6, 7, 8]));
        assert_eq!(state.phase, ControlPhase::Idle);
    }

    #[test]
    fn newer_setup_packet_replaces_stale_pending_state() {
        let mut state = ControlState {
            phase: ControlPhase::SetupPending([1, 2, 3, 4, 5, 6, 7, 8]),
        };

        state.record_setup_packet([8, 7, 6, 5, 4, 3, 2, 1]);

        assert_eq!(state.take_setup_packet(), Some([8, 7, 6, 5, 4, 3, 2, 1]));
    }

    #[test]
    fn pending_setup_takes_priority_over_buffered_out_payload() {
        let mut state = ControlState {
            phase: ControlPhase::SetupPending([1, 2, 3, 4, 5, 6, 7, 8]),
        };
        let pending_control_out = Some((OutCompletion::Data(8), 8));

        assert!(matches!(state.phase, ControlPhase::SetupPending(_)));
        assert!(pending_control_out.is_some());
    }

    #[test]
    fn final_data_out_transitions_to_awaiting_accept() {
        let mut state = ControlState {
            phase: ControlPhase::DataOut { last: true },
        };

        assert_eq!(state.finish_out(12), OutCompletion::AwaitingAccept(12));
        assert_eq!(state.phase, ControlPhase::AwaitingAccept);
    }

    #[test]
    fn non_final_data_out_returns_to_idle() {
        let mut state = ControlState {
            phase: ControlPhase::DataOut { last: false },
        };

        assert_eq!(state.finish_out(8), OutCompletion::Data(8));
        assert_eq!(state.phase, ControlPhase::Idle);
    }

    #[test]
    fn final_data_in_requests_status_stage() {
        let mut state = ControlState {
            phase: ControlPhase::DataIn { last: true },
        };

        assert_eq!(state.finish_in(), InCompletion::NeedsStatusOut);
        assert_eq!(state.phase, ControlPhase::Idle);
    }

    #[test]
    fn non_final_data_in_returns_to_idle() {
        let mut state = ControlState {
            phase: ControlPhase::DataIn { last: false },
        };

        assert_eq!(state.finish_in(), InCompletion::DataComplete);
        assert_eq!(state.phase, ControlPhase::Idle);
    }

    #[test]
    fn status_in_completion_returns_to_idle() {
        let mut state = ControlState {
            phase: ControlPhase::StatusIn,
        };

        assert_eq!(state.finish_in(), InCompletion::StatusComplete);

        assert_eq!(state.phase, ControlPhase::Idle);
    }

    #[test]
    fn status_out_completion_returns_to_idle() {
        let mut state = ControlState {
            phase: ControlPhase::StatusOut,
        };

        assert_eq!(state.finish_out(0), OutCompletion::StatusComplete);

        assert_eq!(state.phase, ControlPhase::Idle);
    }

    #[test]
    fn interrupted_status_in_is_not_treated_as_success() {
        assert_eq!(
            super::status_in_poll_result(ControlPoll::Interrupted),
            Some(Err(EndpointError::Disabled))
        );
    }
}
