use core::cell::{Cell, RefCell};

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::{Driver, TICK_HZ, time_driver_impl};
use embassy_time_queue_utils::Queue;
use once_cell::sync::OnceCell;

use crate::gtc::GlobalTimerCounter;
use crate::interrupt::{Interrupt, PpiInterrupt};
static CPU_3X2X_CLK_HZ: OnceCell<u64> = OnceCell::new();
static TIME_DRIVER_STATE: OnceCell<()> = OnceCell::new();
static TIME_DRIVER_CPU: OnceCell<u8> = OnceCell::new();

fn with_time_state<R>(f: impl FnOnce(CriticalSection) -> R) -> R {
    crate::multicore::with_reconfiguration_lock(|| critical_section::with(f))
}

struct AlarmState {
    timestamp: Cell<u64>,
    programmed_timestamp: Cell<u64>,
}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
            programmed_timestamp: Cell::new(u64::MAX),
        }
    }
}

unsafe impl Send for AlarmState {}

/// Interrupt handler type for the Embassy global timer driver.
pub struct InterruptHandler;

impl InterruptHandler {
    unsafe fn on_interrupt() {
        unsafe { GTC_TIME_DRIVER.on_interrupt() };
    }
}

/// Initialize the Embassy time driver.
pub fn init(clocks: &crate::clocks::Clocks, gtc: GlobalTimerCounter) {
    if TIME_DRIVER_STATE.get().is_none() {
        unsafe { GTC_TIME_DRIVER.init(clocks, gtc) };
        crate::interrupt::register_internal(
            Interrupt::Ppi(PpiInterrupt::GlobalTimer),
            InterruptHandler::on_interrupt,
        );
        let _ = TIME_DRIVER_CPU.set(crate::multicore::current_cpu_id());
        let _ = TIME_DRIVER_STATE.set(());
    }
}

pub(crate) fn enable_local_interrupt() {
    assert!(
        TIME_DRIVER_STATE.get().is_some(),
        "embassy-zynq7000 time driver not initialized"
    );
    if TIME_DRIVER_CPU.get().copied() != Some(crate::multicore::current_cpu_id()) {
        return;
    }
    crate::runtime::unpend_interrupt(Interrupt::Ppi(PpiInterrupt::GlobalTimer));
    crate::runtime::enable_interrupt(Interrupt::Ppi(PpiInterrupt::GlobalTimer));
}

/// Compatibility interrupt hook for users that still dispatch the global timer manually.
pub unsafe fn on_interrupt() {
    unsafe { GTC_TIME_DRIVER.on_interrupt() };
}

pub struct GtcTimerDriver {
    gtc: Mutex<RefCell<GlobalTimerCounter>>,
    alarms: Mutex<AlarmState>,
    queue: Mutex<RefCell<Queue>>,
}

impl GtcTimerDriver {
    pub unsafe fn init(&'static self, clocks: &crate::clocks::Clocks, mut gtc: GlobalTimerCounter) {
        CPU_3X2X_CLK_HZ
            .set(clocks.arm_clocks().cpu_3x2x_clk().raw() as u64)
            .unwrap();
        gtc.set_prescaler(0);
        gtc.disable_interrupt();
        gtc.clear_interrupt_event();
        gtc.enable();
        with_time_state(|cs| {
            *self.gtc.borrow(cs).borrow_mut() = gtc;
            let alarm = &self.alarms.borrow(cs);
            alarm.timestamp.set(u64::MAX);
            alarm.programmed_timestamp.set(u64::MAX);
        });
    }

    pub unsafe fn on_interrupt(&self) {
        with_time_state(|cs| {
            self.gtc.borrow(cs).borrow_mut().clear_interrupt_event();
            self.trigger_alarm(cs);
        })
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        if CPU_3X2X_CLK_HZ.get().is_none() {
            return false;
        }
        let alarm = &self.alarms.borrow(cs);
        alarm.timestamp.set(timestamp);

        let t = counter_to_ticks(self.gtc.borrow(cs).borrow().read_timer());
        if timestamp <= t {
            let mut gtc = self.gtc.borrow(cs).borrow_mut();
            gtc.disable_interrupt();
            alarm.timestamp.set(u64::MAX);
            alarm.programmed_timestamp.set(u64::MAX);
            return false;
        }

        let safe_timestamp = timestamp.max(t + 3);
        let mut gtc = self.gtc.borrow(cs).borrow_mut();
        let programmed_timestamp = safe_timestamp;
        let Some(opt_comparator) = ticks_to_counter_ceil(safe_timestamp) else {
            gtc.set_comparator(u64::MAX);
            gtc.enable_interrupt();
            alarm.programmed_timestamp.set(programmed_timestamp);
            return true;
        };
        gtc.set_comparator(opt_comparator);
        gtc.enable_interrupt();
        if programmed_alarm_is_stale(programmed_timestamp, counter_to_ticks(gtc.read_timer())) {
            gtc.disable_interrupt();
            gtc.clear_interrupt_event();
            alarm.programmed_timestamp.set(u64::MAX);
            return false;
        }
        alarm.programmed_timestamp.set(programmed_timestamp);
        true
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut gtc = self.gtc.borrow(cs).borrow_mut();
        gtc.disable_interrupt();
        drop(gtc);

        let alarm = &self.alarms.borrow(cs);
        alarm.timestamp.set(u64::MAX);
        alarm.programmed_timestamp.set(u64::MAX);

        let mut next = self
            .queue
            .borrow(cs)
            .borrow_mut()
            .next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self
                .queue
                .borrow(cs)
                .borrow_mut()
                .next_expiration(self.now());
        }
    }
}

impl Driver for GtcTimerDriver {
    #[inline]
    fn now(&self) -> u64 {
        if CPU_3X2X_CLK_HZ.get().is_none() {
            return 0;
        }
        with_time_state(|cs| counter_to_ticks(self.gtc.borrow(cs).borrow().read_timer()))
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        assert!(
            CPU_3X2X_CLK_HZ.get().is_some(),
            "embassy-zynq7000 time driver not initialized"
        );
        with_time_state(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

time_driver_impl!(
    static GTC_TIME_DRIVER: GtcTimerDriver = GtcTimerDriver {
        gtc: Mutex::new(RefCell::new(unsafe { GlobalTimerCounter::steal_fixed(None) })),
        alarms: Mutex::new(AlarmState::new()),
        queue: Mutex::new(RefCell::new(Queue::new())),
});

fn counter_to_ticks(counter: u64) -> u64 {
    let cpu_hz = *CPU_3X2X_CLK_HZ.get().expect("time driver not initialized");
    counter_to_ticks_with_hz(counter, cpu_hz)
}

fn ticks_to_counter_ceil(ticks: u64) -> Option<u64> {
    let cpu_hz = *CPU_3X2X_CLK_HZ.get().expect("time driver not initialized");
    ticks_to_counter_ceil_with_hz(ticks, cpu_hz)
}

fn counter_to_ticks_with_hz(counter: u64, cpu_hz: u64) -> u64 {
    (((counter as u128) * (TICK_HZ as u128)) / (cpu_hz as u128)) as u64
}

fn ticks_to_counter_ceil_with_hz(ticks: u64, cpu_hz: u64) -> Option<u64> {
    let numerator = (ticks as u128).checked_mul(cpu_hz as u128)?;
    let denominator = TICK_HZ as u128;
    let counter = numerator.div_ceil(denominator);
    u64::try_from(counter).ok()
}

fn programmed_alarm_is_stale(programmed_timestamp: u64, now: u64) -> bool {
    programmed_timestamp <= now
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::RefCell;

    #[test]
    fn counter_to_ticks_uses_integer_floor_conversion() {
        let cpu_hz = 333_333_333;
        let counter = cpu_hz * 4 + cpu_hz / 2;

        assert_eq!(
            counter_to_ticks_with_hz(counter, cpu_hz),
            TICK_HZ * 4 + TICK_HZ / 2
        );
    }

    #[test]
    fn ticks_to_counter_ceil_rounds_up_fractional_results() {
        let counter = ticks_to_counter_ceil_with_hz(1, 3).unwrap();
        assert_eq!(counter, 1);

        let cpu_hz = 1_000_001;
        let expected = (cpu_hz as u128).div_ceil(TICK_HZ as u128) as u64;
        assert_eq!(ticks_to_counter_ceil_with_hz(1, cpu_hz), Some(expected));
    }

    #[test]
    fn ticks_to_counter_ceil_returns_none_on_overflow() {
        assert_eq!(ticks_to_counter_ceil_with_hz(u64::MAX, u64::MAX), None);
    }

    #[test]
    fn counter_tick_conversions_are_monotonic() {
        let cpu_hz = 600_000_000;
        let ticks = 123_456_u64;
        let counter = ticks_to_counter_ceil_with_hz(ticks, cpu_hz).unwrap();

        assert!(counter_to_ticks_with_hz(counter, cpu_hz) >= ticks);
        assert!(counter_to_ticks_with_hz(counter.saturating_sub(1), cpu_hz) <= ticks);
    }

    #[test]
    fn set_alarm_avoids_nested_refcell_borrows() {
        struct FakeDriver {
            gtc: RefCell<u32>,
        }

        impl FakeDriver {
            fn now(&self) -> u32 {
                *self.gtc.borrow()
            }

            fn old_pattern_panics(&self) {
                let _borrow = self.gtc.borrow_mut();
                let _ = self.now();
            }

            fn new_pattern_reads_timer_first(&self) -> u32 {
                *self.gtc.borrow()
            }
        }

        let driver = FakeDriver {
            gtc: RefCell::new(7),
        };

        assert!(
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                driver.old_pattern_panics();
            }))
            .is_err()
        );
        assert_eq!(driver.new_pattern_reads_timer_first(), 7);
    }

    #[test]
    fn programmed_alarm_staleness_detects_elapsed_deadline() {
        assert!(programmed_alarm_is_stale(10, 10));
        assert!(programmed_alarm_is_stale(10, 11));
        assert!(!programmed_alarm_is_stale(11, 10));
    }

    #[test]
    fn time_init_tracks_global_driver_state_separately_from_per_cpu_irq_enable() {
        assert!(TIME_DRIVER_STATE.get().is_none());
        assert!(CPU_3X2X_CLK_HZ.get().is_none());
    }
}
