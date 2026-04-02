//! Reset and reboot helpers for the Zynq-7000 PS.

use crate::{gic::Cpu, slcr::Slcr};
use zynq7000::slcr::{RebootStatus, reset::ApuWatchdogTarget};

const fn clear_reset_status_value(status: RebootStatus) -> RebootStatus {
    RebootStatus::clear_from(status)
}

const fn soft_reset_status_value(status: RebootStatus) -> RebootStatus {
    RebootStatus::for_soft_reset(status)
}

/// Read the reboot-status register.
pub fn read_reset_status() -> RebootStatus {
    let slcr = unsafe { zynq7000::slcr::Registers::new_mmio_fixed() };
    slcr.read_reboot_status()
}

/// Clear sticky reboot-status bits by writing the retained value back.
pub fn clear_reset_status(status: RebootStatus) {
    unsafe {
        Slcr::with(|slcr| {
            slcr.write_reboot_status(clear_reset_status_value(status));
        });
    }
}

/// Configure which reset target a CPU private watchdog uses when watchdog mode expires.
pub fn set_apu_watchdog_reset_target(cpu: Cpu, target: ApuWatchdogTarget) {
    unsafe {
        Slcr::with(|slcr| {
            slcr.reset_ctrl().modify_rs_awdt(|mut val| {
                match cpu {
                    Cpu::Cpu0 => val.set_apu_wdt_0_reset_target(target),
                    Cpu::Cpu1 => val.set_apu_wdt_1_reset_target(target),
                }
                val
            });
        });
    }
}

/// Trigger a processing-system software reset.
pub fn reboot_system() -> ! {
    unsafe {
        Slcr::with(|slcr| {
            // Preserve the non-status bits as done in existing Zynq board code.
            let reboot = slcr.read_reboot_status();
            slcr.write_reboot_status(soft_reset_status_value(reboot));
            slcr.reset_ctrl().modify_pss(|mut val| {
                val.set_soft_reset(true);
                val
            });
        });
    }

    loop {
        aarch32_cpu::asm::nop();
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn clear_reset_status_value_preserves_the_register_image() {
        let status = RebootStatus::new_with_raw_value(0xDEAD_BEEF);
        assert_eq!(clear_reset_status_value(status).raw_value(), 0xDEAD_BEEF);
    }

    #[test]
    fn soft_reset_status_value_applies_the_existing_workaround_mask() {
        let status = RebootStatus::new_with_raw_value(0xFFFF_FFFF);
        assert_eq!(soft_reset_status_value(status).raw_value(), 0xF0FF_FFFF);
    }
}
