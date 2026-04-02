static LOGGER_INIT_DONE: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Logging helpers for UART-backed Embassy examples.
pub mod uart_blocking {
    use core::cell::{Cell, UnsafeCell};

    use aarch32_cpu::register::Cpsr;
    use embedded_io::Write as _;
    use log::{LevelFilter, set_logger, set_max_level};

    use crate::uart::AnyTx;

    pub struct UartLoggerUnsafeSingleThread {
        skip_in_isr: Cell<bool>,
        tx: UnsafeCell<Option<AnyTx>>,
    }

    unsafe impl Send for UartLoggerUnsafeSingleThread {}
    unsafe impl Sync for UartLoggerUnsafeSingleThread {}

    static UART_LOGGER_UNSAFE_SINGLE_THREAD: UartLoggerUnsafeSingleThread =
        UartLoggerUnsafeSingleThread {
            skip_in_isr: Cell::new(false),
            tx: UnsafeCell::new(None),
        };

    /// Initialize the logger with a blocking UART TX half which does not use locks.
    ///
    /// # Safety
    ///
    /// This logger performs writes without synchronization and is only suitable for single-core
    /// or otherwise externally serialized use.
    pub unsafe fn init_unsafe_single_core(
        tx: impl Into<AnyTx>,
        level: LevelFilter,
        skip_in_isr: bool,
    ) {
        if super::LOGGER_INIT_DONE.swap(true, core::sync::atomic::Ordering::Relaxed) {
            return;
        }
        let opt_tx = unsafe { &mut *UART_LOGGER_UNSAFE_SINGLE_THREAD.tx.get() };
        opt_tx.replace(tx.into());
        UART_LOGGER_UNSAFE_SINGLE_THREAD
            .skip_in_isr
            .set(skip_in_isr);
        set_logger(&UART_LOGGER_UNSAFE_SINGLE_THREAD).unwrap();
        set_max_level(level);
    }

    impl log::Log for UartLoggerUnsafeSingleThread {
        fn enabled(&self, _metadata: &log::Metadata) -> bool {
            true
        }

        fn log(&self, record: &log::Record) {
            if self.skip_in_isr.get() {
                match Cpsr::read().mode().unwrap() {
                    aarch32_cpu::register::cpsr::ProcessorMode::Fiq
                    | aarch32_cpu::register::cpsr::ProcessorMode::Irq => return,
                    _ => {}
                }
            }

            let tx_mut = unsafe { &mut *self.tx.get() }.as_mut();
            if let Some(tx) = tx_mut {
                writeln!(tx, "{} - {}\r", record.level(), record.args()).unwrap();
            }
        }

        fn flush(&self) {
            let tx_mut = unsafe { &mut *self.tx.get() }.as_mut();
            if let Some(tx) = tx_mut {
                tx.flush().unwrap();
            }
        }
    }
}

pub mod rb {
    use core::cell::RefCell;
    use core::fmt::Write as _;
    use core::{future::poll_fn, task::Poll};

    use critical_section::CriticalSection;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::waitqueue::AtomicWaker;
    use log::{LevelFilter, set_logger, set_max_level};
    use ringbuf::{
        StaticRb,
        traits::{Consumer, Observer, Producer},
    };

    const DISCARD_CHUNK: usize = 64;

    pub struct Logger {
        frame_queue: embassy_sync::channel::Channel<CriticalSectionRawMutex, usize, 32>,
        data_buf: critical_section::Mutex<RefCell<heapless::String<4096>>>,
        ring_buf: critical_section::Mutex<RefCell<Option<StaticRb<u8, 4096>>>>,
        drained_waker: AtomicWaker,
    }

    unsafe impl Send for Logger {}
    unsafe impl Sync for Logger {}

    static LOGGER_RB: Logger = Logger {
        frame_queue: embassy_sync::channel::Channel::new(),
        data_buf: critical_section::Mutex::new(RefCell::new(heapless::String::new())),
        ring_buf: critical_section::Mutex::new(RefCell::new(None)),
        drained_waker: AtomicWaker::new(),
    };

    fn with_logger_state<R>(f: impl FnOnce(CriticalSection) -> R) -> R {
        crate::multicore::with_reconfiguration_lock(|| critical_section::with(f))
    }

    impl log::Log for Logger {
        fn enabled(&self, _metadata: &log::Metadata) -> bool {
            true
        }

        fn log(&self, record: &log::Record) {
            with_logger_state(|cs| {
                let ref_buf = self.data_buf.borrow(cs);
                let mut buf = ref_buf.borrow_mut();
                buf.clear();
                let _ = writeln!(buf, "{} - {}\r", record.level(), record.args());
                let rb_ref = self.ring_buf.borrow(cs);
                let mut rb_opt = rb_ref.borrow_mut();
                let rb = rb_opt.as_mut().expect("log call on uninitialized logger");
                let frame = buf.as_bytes();
                if frame.is_empty() || rb.vacant_len() < frame.len() {
                    return;
                }
                if self.frame_queue.try_send(frame.len()).is_err() {
                    return;
                }
                let written = rb.push_slice(frame);
                debug_assert_eq!(written, frame.len());
            });
        }

        fn flush(&self) {}
    }

    impl Logger {
        pub fn frame_queue(
            &self,
        ) -> &embassy_sync::channel::Channel<CriticalSectionRawMutex, usize, 32> {
            &self.frame_queue
        }

        fn pending_bytes(&self) -> usize {
            with_logger_state(|cs| {
                let rb_ref = self.ring_buf.borrow(cs);
                let rb = rb_ref.borrow();
                rb.as_ref().map(|rb| rb.occupied_len()).unwrap_or(0)
            })
        }
    }

    pub fn init(level: LevelFilter) {
        if super::LOGGER_INIT_DONE.swap(true, core::sync::atomic::Ordering::Relaxed) {
            return;
        }
        with_logger_state(|cs| {
            let rb = StaticRb::<u8, 4096>::default();
            let rb_ref = LOGGER_RB.ring_buf.borrow(cs);
            rb_ref.borrow_mut().replace(rb);
        });
        set_logger(&LOGGER_RB).unwrap();
        set_max_level(level);
    }

    pub fn read_next_frame(frame_len: usize, buf: &mut [u8]) {
        let read_len = core::cmp::min(frame_len, buf.len());
        with_logger_state(|cs| {
            let rb_ref = LOGGER_RB.ring_buf.borrow(cs);
            let mut rb = rb_ref.borrow_mut();
            let rb = rb.as_mut().unwrap();
            rb.pop_slice(&mut buf[..read_len]);

            let mut remaining = frame_len.saturating_sub(read_len);
            let mut discard = [0u8; DISCARD_CHUNK];
            while remaining != 0 {
                let chunk = core::cmp::min(remaining, discard.len());
                rb.pop_slice(&mut discard[..chunk]);
                remaining -= chunk;
            }
        });
        if LOGGER_RB.frame_queue.is_empty() && LOGGER_RB.pending_bytes() == 0 {
            LOGGER_RB.drained_waker.wake();
        }
    }

    pub fn get_frame_queue()
    -> &'static embassy_sync::channel::Channel<CriticalSectionRawMutex, usize, 32> {
        LOGGER_RB.frame_queue()
    }

    pub async fn wait_flushed() {
        poll_fn(|cx| {
            LOGGER_RB.drained_waker.register(cx.waker());
            if LOGGER_RB.frame_queue.is_empty() && LOGGER_RB.pending_bytes() == 0 {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }

    #[cfg(test)]
    mod tests {
        use ringbuf::traits::{Consumer, Observer, Producer};

        use super::*;

        #[test]
        fn push_slice_reports_truncated_frame_length() {
            let mut rb = StaticRb::<u8, 4>::default();

            assert_eq!(rb.push_slice(b"abcdef"), 4);
        }

        #[test]
        fn frame_tail_is_discarded_when_destination_is_too_small() {
            let mut rb = StaticRb::<u8, 8>::default();
            rb.push_slice(b"abcd");

            let mut buf = [0u8; 2];
            rb.pop_slice(&mut buf);

            let mut discard = [0u8; DISCARD_CHUNK];
            rb.pop_slice(&mut discard[..2]);

            assert_eq!(&buf, b"ab");
            assert_eq!(rb.occupied_len(), 0);
        }

        #[test]
        fn truncated_frame_is_dropped_before_queueing() {
            let mut rb = StaticRb::<u8, 4>::default();
            assert_eq!(rb.vacant_len(), 4);
            assert!(rb.vacant_len() < 6);
        }
    }
}
