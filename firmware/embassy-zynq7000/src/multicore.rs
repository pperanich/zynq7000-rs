use core::{
    arch::asm,
    hint::spin_loop,
    sync::atomic::{AtomicU8, Ordering, compiler_fence},
};

use crate::{interrupt::Interrupt, runtime, time};

const PAUSE_SGI_ID: usize = 15;
const NO_OWNER: u8 = 0;

static LOCK_OWNER: AtomicU8 = AtomicU8::new(NO_OWNER);
static LOCK_DEPTH: AtomicU8 = AtomicU8::new(0);

#[inline]
pub(crate) fn current_cpu_id() -> u8 {
    (aarch32_cpu::register::mpidr::Mpidr::read().0 & 0x3) as u8
}

#[inline]
fn owner_for_cpu(cpu_id: u8) -> u8 {
    cpu_id + 1
}

#[inline]
fn interrupts_enabled() -> bool {
    let cpsr: u32;
    unsafe {
        asm!("mrs {0}, cpsr", out(reg) cpsr, options(nomem, nostack, preserves_flags));
    }
    (cpsr & (1 << 7)) == 0
}

/// Register the current CPU as participating in embassy-zynq7000 multicore coordination.
///
/// `crate::init()` calls this automatically for the initializing core. Any additional core that
/// may concurrently invoke drivers touching shared global state must call this once after its
/// interrupt/runtime setup is complete.
///
/// This only enables crate-managed interrupt and locking coordination. The Embassy time driver is
/// still bound to the init core, so secondary-core Embassy executors are not supported yet.
pub fn register_current_core() {
    runtime::unpend_interrupt(Interrupt::Sgi(PAUSE_SGI_ID));
    runtime::enable_interrupt(Interrupt::Sgi(PAUSE_SGI_ID));
    time::enable_local_interrupt();
}

pub(crate) fn with_reconfiguration_lock<R>(f: impl FnOnce() -> R) -> R {
    let guard = LockGuard::acquire();
    let result = f();
    drop(guard);
    result
}

pub(crate) fn dispatch_sgi(sgi_id: usize) -> bool {
    if sgi_id != PAUSE_SGI_ID {
        return false;
    }
    true
}

struct LockGuard {
    restore_interrupts: bool,
    outermost: bool,
}

impl LockGuard {
    fn acquire() -> Self {
        let cpu = current_cpu_id();
        let owner = owner_for_cpu(cpu);
        let restore_interrupts = interrupts_enabled();
        if restore_interrupts {
            aarch32_cpu::interrupt::disable();
        }

        if LOCK_OWNER.load(Ordering::Acquire) == owner {
            LOCK_DEPTH.fetch_add(1, Ordering::Relaxed);
            return Self {
                restore_interrupts: false,
                outermost: false,
            };
        }

        loop {
            if LOCK_OWNER
                .compare_exchange(NO_OWNER, owner, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                LOCK_DEPTH.store(1, Ordering::Release);
                return Self {
                    restore_interrupts,
                    outermost: true,
                };
            }

            if restore_interrupts {
                unsafe { aarch32_cpu::interrupt::enable() };
            }
            spin_loop();
            if restore_interrupts {
                aarch32_cpu::interrupt::disable();
            }
        }
    }
}

impl Drop for LockGuard {
    fn drop(&mut self) {
        if !self.outermost {
            LOCK_DEPTH.fetch_sub(1, Ordering::Relaxed);
            return;
        }

        LOCK_DEPTH.store(0, Ordering::Release);
        LOCK_OWNER.store(NO_OWNER, Ordering::Release);
        compiler_fence(Ordering::SeqCst);

        if self.restore_interrupts {
            unsafe { aarch32_cpu::interrupt::enable() };
        }
    }
}
