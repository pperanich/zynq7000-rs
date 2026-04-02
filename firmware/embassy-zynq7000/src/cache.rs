//! Cache maintenance helpers used by the Embassy HAL's USB backend.
use core::sync::atomic::{AtomicBool, Ordering, compiler_fence};

use aarch32_cpu::{
    asm::dsb,
    cache::{
        clean_and_invalidate_data_cache_line_to_poc, clean_data_cache_line_to_poc,
        invalidate_data_cache_line_to_poc,
    },
};
use zynq7000::l2_cache::{CacheSync, MmioRegisters, Registers};

pub const CACHE_LINE_SIZE: usize = 32;
static DMA_CACHE_READY: AtomicBool = AtomicBool::new(false);

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum AlignmentError {
    #[error("alignment error, addresses and lengths must be aligned to 32 byte cache line length")]
    Unaligned,
    #[error("DMA cache maintenance is unavailable")]
    Unavailable,
}

pub(crate) fn mark_dma_cache_ready() {
    DMA_CACHE_READY.store(true, Ordering::Release);
}

pub(crate) fn dma_cache_ready() -> bool {
    DMA_CACHE_READY.load(Ordering::Acquire)
}

fn sync_l2_cache(l2c: &mut MmioRegisters<'static>) {
    l2c.write_cache_sync(CacheSync::new_with_raw_value(0));
    while l2c.read_cache_sync().busy() {}
}

/// Invalidate an address range.
///
/// This function invalidates both the L1 and L2 cache. The L2C must be enabled and set up
/// correctly for this function to work correctly.
///
/// The provided address and the range to invalidate must both be aligned to the 32 byte cache line
/// length.
pub fn invalidate_data_cache_range(addr: u32, len: usize) -> Result<(), AlignmentError> {
    if !dma_cache_ready() {
        return Err(AlignmentError::Unavailable);
    }
    if !addr.is_multiple_of(CACHE_LINE_SIZE as u32) || !len.is_multiple_of(CACHE_LINE_SIZE) {
        return Err(AlignmentError::Unaligned);
    }
    let mut current_addr = addr;
    let end_addr = addr.saturating_add(len as u32);
    let mut l2c = unsafe { Registers::new_mmio_fixed() };

    dsb();
    // Invalidate outer caches lines first, see chapter 3.3.10 of the L2C technical reference
    // manual.
    while current_addr < end_addr {
        l2c.write_invalidate_by_pa(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    sync_l2_cache(&mut l2c);

    // Invalidate inner cache lines.
    current_addr = addr;
    compiler_fence(core::sync::atomic::Ordering::SeqCst);

    while current_addr < end_addr {
        invalidate_data_cache_line_to_poc(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    // Synchronize the cache maintenance.
    dsb();
    Ok(())
}

/// Cleans an address range.
///
/// This function cleans and invalidates both L1
/// and L2 cache. The L2C must be enabled and set up correctly for this function to work correctly.
///
/// Both the address and length to clean and invalidate must be a multiple of the 32 byte cache
/// line.
pub fn clean_data_cache_range(addr: u32, len: usize) -> Result<(), AlignmentError> {
    if !dma_cache_ready() {
        return Err(AlignmentError::Unavailable);
    }
    if !addr.is_multiple_of(32) || !len.is_multiple_of(32) {
        return Err(AlignmentError::Unaligned);
    }

    let end_addr = addr.saturating_add(len as u32);
    let mut current_addr = addr;
    dsb();

    // For details on the following section, see chapter 3.3.10 of the L2C technical reference
    // manual.
    // Clean inner cache lines first.
    while current_addr < end_addr {
        clean_data_cache_line_to_poc(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    dsb();

    // Clean and invalidates outer cache.
    let mut l2c = unsafe { Registers::new_mmio_fixed() };
    current_addr = addr;
    while current_addr < end_addr {
        l2c.write_clean_by_pa(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    sync_l2_cache(&mut l2c);
    compiler_fence(core::sync::atomic::Ordering::SeqCst);
    Ok(())
}

/// Clean and invalidate an address range.
pub fn clean_and_invalidate_data_cache_range(addr: u32, len: usize) -> Result<(), AlignmentError> {
    if !dma_cache_ready() {
        return Err(AlignmentError::Unavailable);
    }
    if !addr.is_multiple_of(CACHE_LINE_SIZE as u32) || !len.is_multiple_of(CACHE_LINE_SIZE) {
        return Err(AlignmentError::Unaligned);
    }

    let end_addr = addr.saturating_add(len as u32);
    let mut current_addr = addr;
    dsb();

    while current_addr < end_addr {
        clean_data_cache_line_to_poc(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    dsb();

    let mut l2c = unsafe { Registers::new_mmio_fixed() };
    current_addr = addr;
    while current_addr < end_addr {
        l2c.write_clean_invalidate_by_pa(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    sync_l2_cache(&mut l2c);

    current_addr = addr;
    compiler_fence(core::sync::atomic::Ordering::SeqCst);

    while current_addr < end_addr {
        clean_and_invalidate_data_cache_line_to_poc(current_addr);
        current_addr = current_addr.saturating_add(CACHE_LINE_SIZE as u32);
    }
    dsb();
    Ok(())
}
