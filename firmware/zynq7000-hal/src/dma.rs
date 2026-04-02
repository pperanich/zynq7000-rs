//! Internal DMA-safety helpers shared by HAL drivers.
use crate::cache::{
    AlignmentError, CACHE_LINE_SIZE, clean_and_invalidate_data_cache_range, clean_data_cache_range,
    invalidate_data_cache_range,
};

/// Cache-line aligned wrapper for controller-owned DMA-visible state.
#[repr(C, align(32))]
pub(crate) struct CacheAligned<T>(pub T);

impl<T> CacheAligned<T> {
    pub const fn new(inner: T) -> Self {
        Self(inner)
    }
}

pub(crate) const fn cache_maintenance_range(addr: usize, len: usize) -> (u32, usize) {
    if len == 0 {
        return (addr as u32, 0);
    }
    let start = addr & !(CACHE_LINE_SIZE - 1);
    let end = (addr + len + CACHE_LINE_SIZE - 1) & !(CACHE_LINE_SIZE - 1);
    (start as u32, end - start)
}

pub(crate) fn prepare_ref_for_device_read<T>(value: &T) -> Result<(), AlignmentError> {
    let (addr, len) = cache_maintenance_range(
        core::ptr::from_ref(value) as usize,
        core::mem::size_of::<T>(),
    );
    if len == 0 {
        return Ok(());
    }
    clean_data_cache_range(addr, len)
}

pub(crate) fn prepare_slice_for_device_read<T>(slice: &[T]) -> Result<(), AlignmentError> {
    let (addr, len) =
        cache_maintenance_range(slice.as_ptr() as usize, core::mem::size_of_val(slice));
    if len == 0 {
        return Ok(());
    }
    clean_data_cache_range(addr, len)
}

pub(crate) fn prepare_slice_for_device_write<T>(slice: &mut [T]) -> Result<(), AlignmentError> {
    let (addr, len) =
        cache_maintenance_range(slice.as_mut_ptr() as usize, core::mem::size_of_val(slice));
    if len == 0 {
        return Ok(());
    }
    clean_and_invalidate_data_cache_range(addr, len)
}

pub(crate) fn complete_slice_from_device_write<T>(slice: &mut [T]) -> Result<(), AlignmentError> {
    let (addr, len) =
        cache_maintenance_range(slice.as_mut_ptr() as usize, core::mem::size_of_val(slice));
    if len == 0 {
        return Ok(());
    }
    invalidate_data_cache_range(addr, len)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_range_is_unchanged() {
        assert_eq!(cache_maintenance_range(0x1234, 0), (0x1234, 0));
    }

    #[test]
    fn rounds_down_start_and_up_end() {
        assert_eq!(cache_maintenance_range(0x1043, 3), (0x1040, 32));
        assert_eq!(cache_maintenance_range(0x1043, 64), (0x1040, 96));
    }
}
