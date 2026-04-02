//! Asynchronous UART receiver (RX) implementation.
use core::{convert::Infallible, future::poll_fn, marker::PhantomData};

use critical_section::Mutex;
use embassy_sync::waitqueue::AtomicWaker;

use super::{FIFO_DEPTH, Rx, RxErrors, TxAsync, TypedUart, UartId, on_interrupt_tx};

/// Upper bound for the interrupt-driven RX ring buffer capacity.
pub const UART_ASYNC_RX_MAX_CAPACITY: usize = 1024;

static UART_RX_WAKERS: [AtomicWaker; 2] = [const { AtomicWaker::new() }; 2];
static RX_CONTEXTS: [Mutex<core::cell::RefCell<RxAsyncContext>>; 2] =
    [const { Mutex::new(core::cell::RefCell::new(RxAsyncContext::new())) }; 2];

/// Call this from the UART interrupt handler once per UART bank.
///
/// It handles both interrupt-driven RX buffering and asynchronous TX completion.
pub fn on_interrupt(peripheral: UartId) {
    let mut buf = [0; FIFO_DEPTH];
    let idx = peripheral as usize;
    let mut rx = unsafe { Rx::steal(peripheral) };
    let result = rx.on_interrupt(&mut buf, true);
    if result.read_bytes() > 0 || result.errors().is_some() {
        critical_section::with(|cs| {
            let mut ctx = RX_CONTEXTS[idx].borrow(cs).borrow_mut();
            if !ctx.active {
                return;
            }
            if let Some(errors) = result.errors() {
                ctx.errors = Some(merge_rx_errors(ctx.errors, errors));
            }
            for byte in buf.iter().take(result.read_bytes()) {
                if !ctx.push(*byte) {
                    ctx.overflowed = true;
                    break;
                }
            }
        });
        UART_RX_WAKERS[idx].wake();
    }
    on_interrupt_tx(peripheral);
}

const fn merge_rx_errors(lhs: Option<RxErrors>, rhs: RxErrors) -> RxErrors {
    match lhs {
        Some(lhs) => lhs.merge(rhs),
        None => rhs,
    }
}

#[derive(Debug)]
struct RxAsyncContext {
    buf: [u8; UART_ASYNC_RX_MAX_CAPACITY],
    head: usize,
    len: usize,
    capacity: usize,
    errors: Option<RxErrors>,
    overflowed: bool,
    active: bool,
}

impl RxAsyncContext {
    const fn new() -> Self {
        Self {
            buf: [0; UART_ASYNC_RX_MAX_CAPACITY],
            head: 0,
            len: 0,
            capacity: 0,
            errors: None,
            overflowed: false,
            active: false,
        }
    }

    fn clear(&mut self, capacity: usize) {
        self.head = 0;
        self.len = 0;
        self.capacity = capacity;
        self.errors = None;
        self.overflowed = false;
        self.active = true;
    }

    fn push(&mut self, byte: u8) -> bool {
        if self.len >= self.capacity {
            return false;
        }
        let idx = (self.head + self.len) % self.capacity;
        self.buf[idx] = byte;
        self.len += 1;
        true
    }

    fn pop_into(&mut self, data: &mut [u8]) -> usize {
        let read = core::cmp::min(self.len, data.len());
        for slot in data.iter_mut().take(read) {
            *slot = self.buf[self.head];
            self.head = (self.head + 1) % self.capacity;
            self.len -= 1;
        }
        read
    }

    fn has_pending_signal(&self) -> bool {
        self.errors.is_some() || self.overflowed
    }
}

/// Buffered asynchronous UART receiver.
pub struct RxAsync<T: super::Instance, const N: usize> {
    rx: Rx,
    _phantom: PhantomData<T>,
}

impl<T: super::Instance, const N: usize> RxAsync<T, N> {
    /// Constructor with a typed interrupt binding.
    pub fn new(
        rx: super::TypedRx<T>,
        _irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, super::InterruptHandler<T>>,
    ) -> Self {
        let mut rx = rx.release();
        assert!(N > 0, "RX async buffer capacity must be non-zero");
        assert!(
            N <= UART_ASYNC_RX_MAX_CAPACITY,
            "RX async buffer capacity exceeds global maximum"
        );
        let idx = rx.uart_idx() as usize;
        rx.start_interrupt_driven_reception();
        critical_section::with(|cs| {
            RX_CONTEXTS[idx].borrow(cs).borrow_mut().clear(N);
        });
        Self {
            rx,
            _phantom: PhantomData,
        }
    }

    /// Read up to `buf.len()` bytes asynchronously.
    pub async fn read(&mut self, buf: &mut [u8]) -> usize {
        if buf.is_empty() {
            return 0;
        }
        let idx = self.rx.uart_idx() as usize;
        poll_fn(|cx| {
            UART_RX_WAKERS[idx].register(cx.waker());
            let (read, has_signal) = critical_section::with(|cs| {
                let mut ctx = RX_CONTEXTS[idx].borrow(cs).borrow_mut();
                let read = ctx.pop_into(buf);
                let has_signal = ctx.has_pending_signal();
                (read, has_signal)
            });
            if read > 0 {
                return core::task::Poll::Ready(read);
            }
            if has_signal {
                return core::task::Poll::Ready(0);
            }
            core::task::Poll::Pending
        })
        .await
    }

    /// Number of buffered bytes currently available.
    pub fn available(&self) -> usize {
        let idx = self.rx.uart_idx() as usize;
        critical_section::with(|cs| RX_CONTEXTS[idx].borrow(cs).borrow().len)
    }

    /// Retrieve and clear sticky RX error flags gathered in the interrupt handler.
    pub fn take_errors(&mut self) -> Option<RxErrors> {
        let idx = self.rx.uart_idx() as usize;
        critical_section::with(|cs| {
            let mut ctx = RX_CONTEXTS[idx].borrow(cs).borrow_mut();
            let errors = ctx.errors;
            ctx.errors = None;
            errors
        })
    }

    /// Returns whether incoming bytes were dropped because the internal ring buffer was full.
    pub fn take_overflowed(&mut self) -> bool {
        let idx = self.rx.uart_idx() as usize;
        critical_section::with(|cs| {
            let mut ctx = RX_CONTEXTS[idx].borrow(cs).borrow_mut();
            let overflowed = ctx.overflowed;
            ctx.overflowed = false;
            overflowed
        })
    }

    /// Release the underlying blocking RX driver.
    pub fn release(mut self) -> super::TypedRx<T> {
        let idx = self.rx.uart_idx() as usize;
        self.rx.disable_interrupts();
        critical_section::with(|cs| {
            RX_CONTEXTS[idx].borrow(cs).borrow_mut().active = false;
        });
        let this = core::mem::ManuallyDrop::new(self);
        super::TypedRx {
            inner: unsafe { core::ptr::read(&this.rx) },
            _phantom: PhantomData,
        }
    }
}

impl<T: super::Instance, const N: usize> Drop for RxAsync<T, N> {
    fn drop(&mut self) {
        let idx = self.rx.uart_idx() as usize;
        self.rx.disable_interrupts();
        critical_section::with(|cs| {
            RX_CONTEXTS[idx].borrow(cs).borrow_mut().active = false;
        });
    }
}

impl<T: super::Instance, const N: usize> embedded_io::ErrorType for RxAsync<T, N> {
    type Error = Infallible;
}

impl<T: super::Instance, const N: usize> embedded_io_async::Read for RxAsync<T, N> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(self.read(buf).await)
    }
}

/// Combined asynchronous UART RX/TX wrapper.
pub struct UartAsync<T: super::Instance, const N: usize> {
    /// Asynchronous transmitter.
    pub tx: TxAsync<T>,
    /// Buffered asynchronous receiver.
    pub rx: RxAsync<T, N>,
}

impl<T: super::Instance, const N: usize> UartAsync<T, N> {
    /// Construct from a blocking UART instance with a typed interrupt binding.
    pub fn new(
        uart: TypedUart<T>,
        irq: impl crate::interrupt::typelevel::Binding<T::Interrupt, super::InterruptHandler<T>> + Copy,
    ) -> Self {
        let (tx, rx) = uart.split();
        Self {
            tx: TxAsync::new(tx, irq),
            rx: RxAsync::new(rx, irq),
        }
    }

    /// Split into asynchronous TX/RX halves.
    pub fn split(self) -> (TxAsync<T>, RxAsync<T, N>) {
        (self.tx, self.rx)
    }
}

impl<T: super::Instance, const N: usize> embedded_io::ErrorType for UartAsync<T, N> {
    type Error = Infallible;
}

impl<T: super::Instance, const N: usize> embedded_io_async::Read for UartAsync<T, N> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(self.rx.read(buf).await)
    }
}

impl<T: super::Instance, const N: usize> embedded_io_async::Write for UartAsync<T, N> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(self.tx.write(buf).await)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush().await
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn pending_signal_is_reported_without_buffered_bytes() {
        let mut ctx = RxAsyncContext::new();
        ctx.clear(8);
        assert!(!ctx.has_pending_signal());

        ctx.errors = Some(RxErrors::default());
        assert!(ctx.has_pending_signal());

        ctx.errors = None;
        ctx.overflowed = true;
        assert!(ctx.has_pending_signal());
    }
}
