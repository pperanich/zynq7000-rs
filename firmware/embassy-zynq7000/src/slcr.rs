use zynq7000::slcr::MmioRegisters;

pub const LOCK_KEY: u32 = 0x767B;
pub const UNLOCK_KEY: u32 = 0xDF0D;

pub(crate) unsafe fn with_unlocked<F: FnOnce(&mut MmioRegisters<'static>)>(f: F) {
    crate::multicore::with_reconfiguration_lock(|| {
        struct SlcrUnlockGuard(*mut MmioRegisters<'static>);

        impl Drop for SlcrUnlockGuard {
            fn drop(&mut self) {
                unsafe { (*self.0).write_lock(LOCK_KEY) };
            }
        }

        let mut slcr = unsafe { zynq7000::slcr::Registers::new_mmio_fixed() };
        slcr.write_unlock(UNLOCK_KEY);
        let _guard = SlcrUnlockGuard(&mut slcr);
        f(&mut slcr);
    });
}
