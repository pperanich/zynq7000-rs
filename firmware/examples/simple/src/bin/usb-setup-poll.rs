//! Vendor-specific USB proof-of-life example.
//!
//! This Phase 2 example avoids UART logging on MIO48/49 so USB0 reset can use MIO48 on
//! Red Pitaya-style boards. It performs enough Chapter 9 handling to enumerate as a
//! vendor-specific device and loops EP1 bulk OUT traffic back over EP1 IN.
#![no_std]
#![no_main]

use aarch32_cpu::asm::nop;
use core::panic::PanicInfo;
use zynq7000_hal::{
    clocks::Clocks,
    gpio::{self, PinState, mio},
    l2_cache,
    time::Hertz,
    usb,
};

const PS_CLOCK_FREQUENCY: Hertz = Hertz::from_raw(33_333_300);
const BULK_PACKET_SIZE: usize = 64;
const VENDOR_REQUEST_GET_STATUS: u8 = 0x5a;
const USB0_RESET_PULSE_CYCLES: usize = 1024;

const DEVICE_DESCRIPTOR: [u8; 18] = [
    18, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 64, 0x09, 0x12, 0x02, 0x00, 0x00, 0x01, 0x01, 0x02,
    0x03, 0x01,
];

const CONFIG_DESCRIPTOR: [u8; 32] = [
    9,
    0x02,
    32,
    0x00,
    0x01,
    0x01,
    0x00,
    0xC0,
    0x00,
    9,
    0x04,
    0x00,
    0x00,
    0x02,
    0xFF,
    0x00,
    0x00,
    0x00,
    7,
    0x05,
    0x01,
    0x02,
    BULK_PACKET_SIZE as u8,
    0x00,
    0x00,
    7,
    0x05,
    0x81,
    0x02,
    BULK_PACKET_SIZE as u8,
    0x00,
    0x00,
];

const LANG_ID_DESCRIPTOR: [u8; 4] = [4, 0x03, 0x09, 0x04];
const VENDOR_STATUS_RESPONSE: [u8; 8] = *b"zynq-usb";

#[derive(Debug, Clone, Copy)]
struct SetupPacket {
    bm_request_type: u8,
    b_request: u8,
    w_value: u16,
    w_index: u16,
    w_length: u16,
}

impl SetupPacket {
    fn from_bytes(bytes: [u8; 8]) -> Self {
        Self {
            bm_request_type: bytes[0],
            b_request: bytes[1],
            w_value: u16::from_le_bytes([bytes[2], bytes[3]]),
            w_index: u16::from_le_bytes([bytes[4], bytes[5]]),
            w_length: u16::from_le_bytes([bytes[6], bytes[7]]),
        }
    }

    fn direction_in(self) -> bool {
        self.bm_request_type & 0x80 != 0
    }

    fn request_type(self) -> u8 {
        self.bm_request_type & 0x60
    }

    fn recipient(self) -> u8 {
        self.bm_request_type & 0x1f
    }

    fn descriptor_type(self) -> u8 {
        (self.w_value >> 8) as u8
    }

    fn descriptor_index(self) -> u8 {
        self.w_value as u8
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PendingStatusAction {
    None,
    SetConfiguration(u8),
}

#[zynq7000_rt::entry]
fn main() -> ! {
    let mut dp = zynq7000::Peripherals::take().unwrap();
    l2_cache::init_with_defaults(&mut dp.l2c);

    let _clocks = Clocks::new_from_regs(PS_CLOCK_FREQUENCY).unwrap();
    let mio_pins = mio::Pins::new(dp.gpio);
    let mut _usb_reset = init_red_pitaya_usb0_reset(mio_pins.mio48);

    let mut resources = usb::UsbResources::new();
    let mut device = usb::UsbController::usb0(dp.usb_0).into_device(&mut resources);
    let mut ep1_out_buffer = usb::AlignedUsbBuffer::<BULK_PACKET_SIZE>::new();
    let mut ep1_in_buffer = usb::AlignedUsbBuffer::<BULK_PACKET_SIZE>::new();
    let mut configured = false;
    let mut configuration_value = 0u8;
    let mut pending_status_action = PendingStatusAction::None;

    device.start_device_mode().unwrap();

    loop {
        let irq = device.poll();
        if irq.reset {
            configured = false;
            configuration_value = 0;
            pending_status_action = PendingStatusAction::None;
            device.on_bus_reset().unwrap();
        }

        if let Some(setup) = device.read_setup_packet().unwrap() {
            let setup = SetupPacket::from_bytes(setup);
            let result = handle_setup_packet(
                &mut device,
                setup,
                configuration_value,
                &mut pending_status_action,
            );
            if result.is_err() {
                device.stall_ep0().unwrap();
                pending_status_action = PendingStatusAction::None;
            }
        }

        if device
            .take_transfer_complete(usb_ep(0, embassy_usb_driver::Direction::Out))
            .unwrap()
            .is_some()
        {}

        if device
            .take_transfer_complete(usb_ep(0, embassy_usb_driver::Direction::In))
            .unwrap()
            .is_some()
        {
            match pending_status_action {
                PendingStatusAction::None => {}
                PendingStatusAction::SetConfiguration(value) => {
                    apply_configuration(
                        &mut device,
                        &mut ep1_out_buffer,
                        value,
                        &mut configured,
                        &mut configuration_value,
                    )
                    .unwrap();
                    pending_status_action = PendingStatusAction::None;
                }
            }
        }

        if configured {
            if let Some(report) = device
                .take_transfer_complete(usb_ep(1, embassy_usb_driver::Direction::Out))
                .unwrap()
            {
                device.finish_out(&mut ep1_out_buffer).unwrap();
                if report.actual_bytes > 0 {
                    ep1_in_buffer.0[..report.actual_bytes]
                        .copy_from_slice(&ep1_out_buffer.0[..report.actual_bytes]);
                    match device.prime_in(
                        usb_ep(1, embassy_usb_driver::Direction::In),
                        &ep1_in_buffer,
                        report.actual_bytes,
                    ) {
                        Ok(()) | Err(usb::UsbError::EndpointBusy(_)) => {}
                        Err(err) => panic_with_error(err),
                    }
                }
                device
                    .prime_out(
                        usb_ep(1, embassy_usb_driver::Direction::Out),
                        &mut ep1_out_buffer,
                        BULK_PACKET_SIZE,
                    )
                    .unwrap();
            }

            if device
                .take_transfer_complete(usb_ep(1, embassy_usb_driver::Direction::In))
                .unwrap()
                .is_some()
            {}
        }
    }
}

fn handle_setup_packet(
    device: &mut usb::UsbDevice<'_>,
    setup: SetupPacket,
    configuration_value: u8,
    pending_status_action: &mut PendingStatusAction,
) -> Result<(), usb::UsbError> {
    *pending_status_action = PendingStatusAction::None;

    match (setup.request_type(), setup.direction_in(), setup.b_request) {
        (0x00, true, 0x06) => {
            handle_get_descriptor(device, setup)?;
            device.prime_ep0_out_status()?;
            Ok(())
        }
        (0x00, false, 0x05) => {
            device.arm_address_after_status(setup.w_value as u8);
            device.prime_ep0_in_status()
        }
        (0x00, false, 0x09) => {
            let value = setup.w_value as u8;
            if value > 1 {
                return Err(usb::UsbError::UnsupportedRequest);
            }
            *pending_status_action = PendingStatusAction::SetConfiguration(value);
            device.prime_ep0_in_status()
        }
        (0x00, true, 0x08) => {
            device.prime_ep0_in(&[configuration_value], setup.w_length as usize)?;
            device.prime_ep0_out_status()?;
            Ok(())
        }
        (0x00, true, 0x00) => {
            let status = match setup.recipient() {
                0x00 | 0x01 | 0x02 => [0u8, 0u8],
                _ => return Err(usb::UsbError::UnsupportedRequest),
            };
            device.prime_ep0_in(&status, setup.w_length as usize)?;
            device.prime_ep0_out_status()?;
            Ok(())
        }
        (0x00, true, 0x0a) => {
            device.prime_ep0_in(&[0u8], setup.w_length as usize)?;
            device.prime_ep0_out_status()?;
            Ok(())
        }
        (0x00, false, 0x0b) if setup.w_value == 0 && setup.w_index == 0 => {
            device.prime_ep0_in_status()
        }
        (0x40, true, VENDOR_REQUEST_GET_STATUS) => {
            device.prime_ep0_in(&VENDOR_STATUS_RESPONSE, setup.w_length as usize)?;
            device.prime_ep0_out_status()?;
            Ok(())
        }
        _ => Err(usb::UsbError::UnsupportedRequest),
    }
}

fn handle_get_descriptor(
    device: &mut usb::UsbDevice<'_>,
    setup: SetupPacket,
) -> Result<(), usb::UsbError> {
    match (setup.descriptor_type(), setup.descriptor_index()) {
        (0x01, 0) => {
            device.prime_ep0_in(&DEVICE_DESCRIPTOR, setup.w_length as usize)?;
            Ok(())
        }
        (0x02, 0) => {
            device.prime_ep0_in(&CONFIG_DESCRIPTOR, setup.w_length as usize)?;
            Ok(())
        }
        (0x03, 0) => {
            device.prime_ep0_in(&LANG_ID_DESCRIPTOR, setup.w_length as usize)?;
            Ok(())
        }
        (0x03, 1) => {
            let mut buf = [0u8; 64];
            let len = write_string_descriptor("zynq7000-rs", &mut buf);
            device.prime_ep0_in(&buf[..len], setup.w_length as usize)?;
            Ok(())
        }
        (0x03, 2) => {
            let mut buf = [0u8; 64];
            let len = write_string_descriptor("Phase2 USB", &mut buf);
            device.prime_ep0_in(&buf[..len], setup.w_length as usize)?;
            Ok(())
        }
        (0x03, 3) => {
            let mut buf = [0u8; 64];
            let len = write_string_descriptor("0001", &mut buf);
            device.prime_ep0_in(&buf[..len], setup.w_length as usize)?;
            Ok(())
        }
        _ => Err(usb::UsbError::UnsupportedRequest),
    }
}

fn apply_configuration(
    device: &mut usb::UsbDevice<'_>,
    ep1_out_buffer: &mut usb::AlignedUsbBuffer<BULK_PACKET_SIZE>,
    value: u8,
    configured: &mut bool,
    configuration_value: &mut u8,
) -> Result<(), usb::UsbError> {
    if value == 0 {
        device.configure_endpoint(
            usb_ep(1, embassy_usb_driver::Direction::Out),
            embassy_usb_driver::EndpointType::Bulk,
            BULK_PACKET_SIZE as u16,
            false,
        )?;
        device.configure_endpoint(
            usb_ep(1, embassy_usb_driver::Direction::In),
            embassy_usb_driver::EndpointType::Bulk,
            BULK_PACKET_SIZE as u16,
            false,
        )?;
        *configured = false;
        *configuration_value = 0;
        return Ok(());
    }

    device.configure_endpoint(
        usb_ep(1, embassy_usb_driver::Direction::Out),
        embassy_usb_driver::EndpointType::Bulk,
        BULK_PACKET_SIZE as u16,
        true,
    )?;
    device.configure_endpoint(
        usb_ep(1, embassy_usb_driver::Direction::In),
        embassy_usb_driver::EndpointType::Bulk,
        BULK_PACKET_SIZE as u16,
        true,
    )?;
    device.prime_out(
        usb_ep(1, embassy_usb_driver::Direction::Out),
        ep1_out_buffer,
        BULK_PACKET_SIZE,
    )?;
    *configured = true;
    *configuration_value = value;
    Ok(())
}

fn write_string_descriptor(input: &str, out: &mut [u8]) -> usize {
    let utf16_len = input.len().min((out.len().saturating_sub(2)) / 2);
    let total_len = 2 + utf16_len * 2;
    out[0] = total_len as u8;
    out[1] = 0x03;
    for (idx, byte) in input.as_bytes().iter().copied().take(utf16_len).enumerate() {
        out[2 + idx * 2] = byte;
        out[3 + idx * 2] = 0;
    }
    total_len
}

fn usb_ep(index: usize, dir: embassy_usb_driver::Direction) -> embassy_usb_driver::EndpointAddress {
    embassy_usb_driver::EndpointAddress::from_parts(index, dir)
}

fn init_red_pitaya_usb0_reset(pin: mio::Pin<mio::Mio48>) -> gpio::Output {
    let mut reset = gpio::Output::new_for_mio(pin, PinState::Low);
    for _ in 0..USB0_RESET_PULSE_CYCLES {
        nop();
    }
    let _ = reset.set_high();
    reset
}

fn panic_with_error(_err: usb::UsbError) -> ! {
    loop {
        nop();
    }
}

#[zynq7000_rt::irq]
fn irq_handler() {}

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
fn panic(_info: &PanicInfo) -> ! {
    loop {
        nop();
    }
}
