use zynq7000::gic::{
    CpuInterfaceRegisters, DistributorControlRegister, DistributorRegisters, InterfaceControl,
    InterruptProcessorTargetRegister, PriorityRegister,
};

use crate::{
    InterruptConfig,
    interrupt::{Interrupt, InterruptInfo, PpiInterrupt, SpiInterrupt},
};

const ICFR_2_FIXED_VALUE: u32 = 0b01010101010111010101010001011111;
const ICFR_3_FIXED_VALUE: u32 = 0b01010101010101011101010101010101;
const ICFR_4_FIXED_VALUE: u32 = 0b01110101010101010101010101010101;
const ICFR_5_FIXED_VALUE: u32 = 0b00000011010101010101010101010101;
const TARGETS_ALL_CPU_0_IPTR_VAL: InterruptProcessorTargetRegister =
    InterruptProcessorTargetRegister::new_with_raw_value(0x01010101);
const MANAGED_INTERRUPTS: [Interrupt; 19] = [
    Interrupt::Sgi(15),
    Interrupt::Ppi(PpiInterrupt::GlobalTimer),
    Interrupt::Ppi(PpiInterrupt::NFiq),
    Interrupt::Ppi(PpiInterrupt::CpuPrivateTimer),
    Interrupt::Ppi(PpiInterrupt::Awdt),
    Interrupt::Ppi(PpiInterrupt::NIrq),
    Interrupt::Spi(SpiInterrupt::Usb0),
    Interrupt::Spi(SpiInterrupt::Eth0),
    Interrupt::Spi(SpiInterrupt::Sdio0),
    Interrupt::Spi(SpiInterrupt::I2c0),
    Interrupt::Spi(SpiInterrupt::Spi0),
    Interrupt::Spi(SpiInterrupt::Uart0),
    Interrupt::Spi(SpiInterrupt::Ttc10),
    Interrupt::Spi(SpiInterrupt::Usb1),
    Interrupt::Spi(SpiInterrupt::Eth1),
    Interrupt::Spi(SpiInterrupt::Sdio1),
    Interrupt::Spi(SpiInterrupt::I2c1),
    Interrupt::Spi(SpiInterrupt::Spi1),
    Interrupt::Spi(SpiInterrupt::Uart1),
];

pub(crate) fn initialize(interrupt_config: InterruptConfig) {
    let mut gicc = unsafe { CpuInterfaceRegisters::new_mmio_fixed() };
    let mut gicd = unsafe { DistributorRegisters::new_mmio_fixed() };
    gicc.write_pmr(PriorityRegister::new_with_raw_value(0xff));
    match interrupt_config {
        InterruptConfig::AllInterruptsToCpu0 => {}
    }

    gicc.write_icr(
        InterfaceControl::builder()
            .with_sbpr(false)
            .with_fiq_en(false)
            .with_ack_ctrl(false)
            .with_enable_non_secure(true)
            .with_enable_secure(true)
            .build(),
    );
    gicd.write_dcr(
        DistributorControlRegister::builder()
            .with_enable_non_secure(true)
            .with_enable_secure(true)
            .build(),
    );
    reset_managed_interrupt_state();
    enable_interrupt(Interrupt::Sgi(15));
}

fn reset_managed_interrupt_state() {
    for interrupt in MANAGED_INTERRUPTS {
        disable_interrupt(interrupt);
        unpend_interrupt(interrupt);
    }
}

pub(crate) fn enable_cpu_interrupts() {
    unsafe {
        aarch32_cpu::interrupt::enable();
    }
}

pub(crate) fn enable_interrupt(interrupt: Interrupt) {
    let mut gicd = unsafe { DistributorRegisters::new_mmio_fixed() };
    configure_interrupt(&mut gicd, interrupt);
    match interrupt {
        Interrupt::Sgi(id) => {
            if id >= 16 {
                return;
            }
            let mask = 1u32 << id;
            gicd.write_icpr(0, mask).unwrap();
            gicd.modify_iser(0, |v| v | mask).unwrap();
        }
        Interrupt::Ppi(ppi) => {
            let mask = 1u32 << (ppi as u32);
            gicd.write_icpr(0, mask).unwrap();
            gicd.modify_iser(0, |v| v | mask).unwrap();
        }
        Interrupt::Spi(spi) => {
            let spi_raw = spi as u32;
            let (reg_idx, bit_pos) = match spi_raw {
                32..=63 => (1usize, spi_raw - 32),
                64..=92 => (2usize, spi_raw - 64),
                _ => return,
            };
            let mask = 1u32 << bit_pos;
            gicd.write_icpr(reg_idx, mask).unwrap();
            gicd.modify_iser(reg_idx, |v| v | mask).unwrap();
        }
        Interrupt::Invalid(_) | Interrupt::Spurious => {}
    }
}

fn configure_interrupt(
    gicd: &mut zynq7000::gic::MmioDistributorRegisters<'static>,
    interrupt: Interrupt,
) {
    match interrupt {
        Interrupt::Sgi(_) | Interrupt::Ppi(_) => {}
        Interrupt::Spi(spi) => {
            let spi_raw = spi as u8;
            let spi_offset = (spi_raw as usize).saturating_sub(32);
            gicd.modify_iptr_spi(spi_offset / 4, |mut v| {
                v.set_targets(spi_offset % 4, TARGETS_ALL_CPU_0_IPTR_VAL.targets(0));
                v
            })
            .unwrap();

            let (register, mask, value) = spi_trigger_config(spi_raw);
            let next = match register {
                2 => (gicd.read_icfr_2_spi() & !mask) | value,
                3 => (gicd.read_icfr_3_spi() & !mask) | value,
                4 => (gicd.read_icfr_4_spi() & !mask) | value,
                5 => (gicd.read_icfr_5_spi() & !mask) | value,
                _ => return,
            };
            match register {
                2 => gicd.write_icfr_2_spi(next),
                3 => gicd.write_icfr_3_spi(next),
                4 => gicd.write_icfr_4_spi(next),
                5 => gicd.write_icfr_5_spi(next),
                _ => unreachable!(),
            }
        }
        Interrupt::Invalid(_) | Interrupt::Spurious => {}
    }
}

fn spi_trigger_config(spi_raw: u8) -> (u8, u32, u32) {
    let (register, fixed) = match spi_raw {
        32..=47 => (2, ICFR_2_FIXED_VALUE),
        48..=63 => (3, ICFR_3_FIXED_VALUE),
        64..=79 => (4, ICFR_4_FIXED_VALUE),
        80..=92 => (5, ICFR_5_FIXED_VALUE),
        _ => return (0, 0, 0),
    };
    let shift = ((spi_raw as u32) & 0x0f) * 2;
    let mask = 0b11_u32 << shift;
    (register, mask, fixed & mask)
}

pub(crate) fn disable_interrupt(interrupt: Interrupt) {
    let mut gicd = unsafe { DistributorRegisters::new_mmio_fixed() };
    match interrupt {
        Interrupt::Sgi(id) => {
            if id >= 16 {
                return;
            }
            let mask = 1u32 << id;
            gicd.modify_icer(0, |v| v | mask).unwrap();
        }
        Interrupt::Ppi(ppi) => {
            let mask = 1u32 << (ppi as u32);
            gicd.modify_icer(0, |v| v | mask).unwrap();
        }
        Interrupt::Spi(spi) => {
            let spi_raw = spi as u32;
            let (reg_idx, bit_pos) = match spi_raw {
                32..=63 => (1usize, spi_raw - 32),
                64..=92 => (2usize, spi_raw - 64),
                _ => return,
            };
            let mask = 1u32 << bit_pos;
            gicd.modify_icer(reg_idx, |v| v | mask).unwrap();
        }
        Interrupt::Invalid(_) | Interrupt::Spurious => {}
    }
}

pub(crate) fn unpend_interrupt(interrupt: Interrupt) {
    let mut gicd = unsafe { DistributorRegisters::new_mmio_fixed() };
    match interrupt {
        Interrupt::Sgi(id) => {
            if id >= 16 {
                return;
            }
            let mask = 1u32 << id;
            gicd.write_icpr(0, mask).unwrap();
        }
        Interrupt::Ppi(ppi) => {
            let mask = 1u32 << (ppi as u32);
            gicd.write_icpr(0, mask).unwrap();
        }
        Interrupt::Spi(spi) => {
            let spi_raw = spi as u32;
            let (reg_idx, bit_pos) = match spi_raw {
                32..=63 => (1usize, spi_raw - 32),
                64..=92 => (2usize, spi_raw - 64),
                _ => return,
            };
            let mask = 1u32 << bit_pos;
            gicd.write_icpr(reg_idx, mask).unwrap();
        }
        Interrupt::Invalid(_) | Interrupt::Spurious => {}
    }
}

pub(crate) fn dispatch_interrupts(mut dispatch: impl FnMut(Interrupt) -> bool) -> bool {
    let mut gicc = unsafe { CpuInterfaceRegisters::new_mmio_fixed() };
    let iar = gicc.read_iar();
    let irq_info = InterruptInfo::new(iar);
    let interrupt = irq_info.interrupt();
    let handled = match interrupt {
        Interrupt::Sgi(id) => crate::multicore::dispatch_sgi(id),
        Interrupt::Spurious | Interrupt::Invalid(_) => false,
        interrupt => dispatch(interrupt),
    };
    if cfg!(debug_assertions) && interrupt.is_supported() && !handled {
        panic!("no registered handler for interrupt: {:?}", interrupt);
    }
    gicc.write_eoir(iar);
    handled
}
