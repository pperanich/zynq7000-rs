use core::cell::{Cell, RefCell};

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::{Driver, TICK_HZ, time_driver_impl};
use embassy_time_queue_utils::Queue;
use once_cell::sync::OnceCell;

use zynq7000_hal::{clocks::ArmClocks, gtc::GlobalTimerCounter, interrupt::typelevel, time::Hertz};

static SCALE: OnceCell<u64> = OnceCell::new();
static CPU_3X2X_CLK: OnceCell<Hertz> = OnceCell::new();

struct AlarmState {
    timestamp: Cell<u64>,
}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
        }
    }
}

unsafe impl Send for AlarmState {}

/// Interrupt handler type for the Embassy global timer driver.
pub struct InterruptHandler;

impl typelevel::Handler<typelevel::GlobalTimer> for InterruptHandler {
    unsafe fn on_interrupt() {
        unsafe { GTC_TIME_DRIVER.on_interrupt() };
    }
}

/// Initialize the Embassy time driver.
///
/// This low-level entry point is kept for compatibility with existing users.
pub fn init(arm_clocks: &ArmClocks, gtc: GlobalTimerCounter) {
    if SCALE.get().is_some() || CPU_3X2X_CLK.get().is_some() {
        return;
    }
    unsafe { GTC_TIME_DRIVER.init(arm_clocks, gtc) };
}

/// Compatibility interrupt hook for users that still dispatch the global timer manually.
///
/// Prefer binding [`InterruptHandler`] via `zynq7000_embassy::bind_interrupts!`.
pub unsafe fn on_interrupt() {
    unsafe { GTC_TIME_DRIVER.on_interrupt() };
}

pub struct GtcTimerDriver {
    gtc: Mutex<RefCell<GlobalTimerCounter>>,
    alarms: Mutex<AlarmState>,
    queue: Mutex<RefCell<Queue>>,
}

impl GtcTimerDriver {
    pub unsafe fn init(&'static self, arm_clock: &ArmClocks, mut gtc: GlobalTimerCounter) {
        CPU_3X2X_CLK.set(arm_clock.cpu_3x2x_clk()).unwrap();
        SCALE
            .set(arm_clock.cpu_3x2x_clk().raw() as u64 / TICK_HZ)
            .unwrap();
        gtc.set_cpu_3x2x_clock(arm_clock.cpu_3x2x_clk());
        gtc.set_prescaler(0);
        gtc.enable();
        critical_section::with(|cs| {
            *self.gtc.borrow(cs).borrow_mut() = gtc;
        });
    }

    pub unsafe fn on_interrupt(&self) {
        critical_section::with(|cs| {
            self.trigger_alarm(cs);
        })
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        if SCALE.get().is_none() {
            return false;
        }
        let mut gtc = self.gtc.borrow(cs).borrow_mut();
        let alarm = &self.alarms.borrow(cs);
        alarm.timestamp.set(timestamp);

        let t = self.now();
        if timestamp <= t {
            gtc.disable_interrupt();
            alarm.timestamp.set(u64::MAX);
            return false;
        }

        let safe_timestamp = timestamp.max(t + 3);
        let opt_comparator = safe_timestamp.checked_mul(*SCALE.get().unwrap());
        if opt_comparator.is_none() {
            return true;
        }
        gtc.set_comparator(opt_comparator.unwrap());
        gtc.enable_interrupt();
        true
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut gtc = self.gtc.borrow(cs).borrow_mut();
        gtc.disable_interrupt();
        drop(gtc);

        let alarm = &self.alarms.borrow(cs);
        alarm.timestamp.set(u64::MAX);

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
        if SCALE.get().is_none() {
            return 0;
        }
        critical_section::with(|cs| self.gtc.borrow(cs).borrow().read_timer())
            / SCALE.get().unwrap()
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
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
