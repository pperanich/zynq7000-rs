use crate::{
    Config, InitError, InterruptConfig, L2CacheMode, Peripherals, clocks, l2_cache, runtime, slcr,
    time,
};

pub(crate) fn init(config: Config) -> Result<Peripherals, InitError> {
    let peripherals = Peripherals::take();
    let clocks = clocks::read_from_regs(config.ps_clock_frequency)?;
    let mut raw = crate::pac::Peripherals::take().ok_or(InitError::PeripheralsAlreadyTaken)?;

    apply_level_shifter_policy(config);
    runtime::initialize(
        config
            .interrupt_config
            .unwrap_or(InterruptConfig::AllInterruptsToCpu0),
    );

    match config.l2_cache_mode {
        L2CacheMode::Initialize => {
            l2_cache::init_with_defaults(&mut raw.l2c);
            crate::cache::mark_dma_cache_ready();
        }
        L2CacheMode::AssumeInitializedForDma => crate::cache::mark_dma_cache_ready(),
    }

    clocks::init(clocks);
    initialize_time_driver();
    crate::multicore::register_current_core();
    runtime::enable_cpu_interrupts();

    drop(raw);
    Ok(peripherals)
}

fn apply_level_shifter_policy(config: Config) {
    let Some(level_shifter_config) = config.level_shifter_config else {
        return;
    };

    unsafe {
        slcr::with_unlocked(|slcr| {
            slcr.write_lvl_shftr_en(crate::pac::slcr::LevelShifterRegister::new_with_raw_value(
                level_shifter_config as u32,
            ));
        });
    }
}

fn initialize_time_driver() {
    let frozen_clocks = clocks::get();
    let gtc = crate::gtc::GlobalTimerCounter::new(frozen_clocks.arm_clocks());
    time::init(frozen_clocks, gtc);
}
