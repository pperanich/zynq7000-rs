use core::sync::atomic::{Ordering, compiler_fence};

use arbitrary_int::{u2, u3};
use zynq7000::l2_cache::{
    Associativity, AuxControl, CacheSync, Control, InterruptControl, LatencyConfig, MmioRegisters,
    ReplacementPolicy, WaySize,
};

use crate::slcr;

const AUX_CTRL_DEFAULT: AuxControl = AuxControl::builder()
    .with_early_bresp_enable(true)
    .with_isntruction_prefetch_enable(true)
    .with_data_prefetch_enable(true)
    .with_nonsec_interrupt_access_control(false)
    .with_nonsec_lockdown_enable(false)
    .with_cache_replace_policy(ReplacementPolicy::RoundRobin)
    .with_force_write_alloc(u2::new(0))
    .with_shared_attr_override(false)
    .with_parity_enable(true)
    .with_event_monitor_bus_enable(true)
    .with_way_size(WaySize::_64kB)
    .with_associativity(Associativity::_8Way)
    .with_shared_attribute_invalidate(false)
    .with_exclusive_cache_config(false)
    .with_store_buff_device_limitation_enable(false)
    .with_high_priority_so_dev_reads(false)
    .with_full_line_zero_enable(false)
    .build();

const DEFAULT_TAG_RAM_LATENCY: LatencyConfig = LatencyConfig::builder()
    .with_write_access_latency(u3::new(0b001))
    .with_read_access_latency(u3::new(0b001))
    .with_setup_latency(u3::new(0b001))
    .build();

const DEFAULT_DATA_RAM_LATENCY: LatencyConfig = LatencyConfig::builder()
    .with_write_access_latency(u3::new(0b001))
    .with_read_access_latency(u3::new(0b010))
    .with_setup_latency(u3::new(0b001))
    .build();

const SLCR_L2C_CONFIG_MAGIC_VALUE: u32 = 0x0002_0202;

pub(crate) fn init_with_defaults(l2c_mmio: &mut MmioRegisters<'static>) {
    l2c_mmio.write_control(Control::new_disabled());
    l2c_mmio.write_aux_control(AUX_CTRL_DEFAULT);
    l2c_mmio.write_tag_ram_latency(DEFAULT_TAG_RAM_LATENCY);
    l2c_mmio.write_data_ram_latency(DEFAULT_DATA_RAM_LATENCY);
    l2c_mmio.write_clean_invalidate_by_way(0xffff);
    l2c_mmio.write_cache_sync(CacheSync::new_with_raw_value(0));
    while l2c_mmio.read_cache_sync().busy() {}
    compiler_fence(Ordering::SeqCst);

    let pending = l2c_mmio.read_interrupt_raw_status();
    l2c_mmio.write_interrupt_clear(InterruptControl::clear_from_status(pending));
    unsafe {
        slcr::with_unlocked(|slcr| {
            slcr.write_magic_l2c_register(SLCR_L2C_CONFIG_MAGIC_VALUE);
        });
    }
    l2c_mmio.write_control(Control::new_enabled());
}
