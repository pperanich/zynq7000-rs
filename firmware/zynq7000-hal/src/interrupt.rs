//! Type-level interrupt bindings for Zynq-7000 drivers.

use crate::gic::Interrupt;

#[cfg(test)]
use crate::gic::{PpiInterrupt, SpiInterrupt};

pub mod typelevel {
    //! Type-level interrupt and binding traits.

    use crate::gic::Interrupt as GicInterrupt;

    mod sealed {
        pub trait SealedInterrupt {}
    }

    /// Type-level logical interrupt.
    pub trait Interrupt: sealed::SealedInterrupt {
        /// Concrete GIC interrupt routed for this source.
        const IRQ: GicInterrupt;
    }

    /// Driver interrupt handler.
    pub trait Handler<I: Interrupt> {
        /// Handle one interrupt delivery for `I`.
        ///
        /// # Safety
        ///
        /// Must only be called from the interrupt context corresponding to `I`.
        unsafe fn on_interrupt();
    }

    /// Compile-time proof that `I` is bound to `H`.
    ///
    /// # Safety
    ///
    /// Implementers assert that `H::on_interrupt()` will run whenever `I` fires.
    pub unsafe trait Binding<I: Interrupt, H: Handler<I>>: Copy {}

    /// Global timer interrupt source.
    pub enum GlobalTimer {}
    impl sealed::SealedInterrupt for GlobalTimer {}
    impl Interrupt for GlobalTimer {
        const IRQ: GicInterrupt = GicInterrupt::Ppi(crate::gic::PpiInterrupt::GlobalTimer);
    }

    /// USB0 interrupt source.
    pub enum Usb0 {}
    impl sealed::SealedInterrupt for Usb0 {}
    impl Interrupt for Usb0 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(crate::gic::SpiInterrupt::Usb0);
    }

    /// USB1 interrupt source.
    pub enum Usb1 {}
    impl sealed::SealedInterrupt for Usb1 {}
    impl Interrupt for Usb1 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(crate::gic::SpiInterrupt::Usb1);
    }

    /// UART0 interrupt source.
    pub enum Uart0 {}
    impl sealed::SealedInterrupt for Uart0 {}
    impl Interrupt for Uart0 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(crate::gic::SpiInterrupt::Uart0);
    }

    /// UART1 interrupt source.
    pub enum Uart1 {}
    impl sealed::SealedInterrupt for Uart1 {}
    impl Interrupt for Uart1 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(crate::gic::SpiInterrupt::Uart1);
    }

    /// SPI0 interrupt source.
    pub enum Spi0 {}
    impl sealed::SealedInterrupt for Spi0 {}
    impl Interrupt for Spi0 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(crate::gic::SpiInterrupt::Spi0);
    }

    /// SPI1 interrupt source.
    pub enum Spi1 {}
    impl sealed::SealedInterrupt for Spi1 {}
    impl Interrupt for Spi1 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(crate::gic::SpiInterrupt::Spi1);
    }

    /// I2C0 interrupt source.
    pub enum I2c0 {}
    impl sealed::SealedInterrupt for I2c0 {}
    impl Interrupt for I2c0 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(crate::gic::SpiInterrupt::I2c0);
    }

    /// I2C1 interrupt source.
    pub enum I2c1 {}
    impl sealed::SealedInterrupt for I2c1 {}
    impl Interrupt for I2c1 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(crate::gic::SpiInterrupt::I2c1);
    }

    /// Ethernet 0 interrupt source.
    pub enum Eth0 {}
    impl sealed::SealedInterrupt for Eth0 {}
    impl Interrupt for Eth0 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(crate::gic::SpiInterrupt::Eth0);
    }

    /// Ethernet 1 interrupt source.
    pub enum Eth1 {}
    impl sealed::SealedInterrupt for Eth1 {}
    impl Interrupt for Eth1 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(crate::gic::SpiInterrupt::Eth1);
    }
}

/// Check whether `interrupt` matches the type-level source `I`.
pub fn matches<I: typelevel::Interrupt>(interrupt: Interrupt) -> bool {
    interrupt == I::IRQ
}

/// Dispatch one interrupt to a concrete handler if it matches `I`.
///
/// Returns `true` if the handler ran.
///
/// # Safety
///
/// The caller must ensure `interrupt` was delivered in the correct interrupt context.
pub unsafe fn dispatch<I, H>(interrupt: Interrupt) -> bool
where
    I: typelevel::Interrupt,
    H: typelevel::Handler<I>,
{
    if matches::<I>(interrupt) {
        unsafe { H::on_interrupt() };
        true
    } else {
        false
    }
}

#[cfg(test)]
mod tests {
    use core::sync::atomic::{AtomicUsize, Ordering};

    use super::*;

    static CALLS: AtomicUsize = AtomicUsize::new(0);

    struct TestHandler;

    impl typelevel::Handler<typelevel::GlobalTimer> for TestHandler {
        unsafe fn on_interrupt() {
            CALLS.fetch_add(1, Ordering::Relaxed);
        }
    }

    #[test]
    fn matches_global_timer() {
        assert!(matches::<typelevel::GlobalTimer>(Interrupt::Ppi(
            PpiInterrupt::GlobalTimer,
        )));
        assert!(!matches::<typelevel::GlobalTimer>(Interrupt::Spi(
            SpiInterrupt::Usb0,
        )));
    }

    #[test]
    fn dispatches_matching_handler() {
        CALLS.store(0, Ordering::Relaxed);
        let dispatched = unsafe {
            dispatch::<typelevel::GlobalTimer, TestHandler>(Interrupt::Ppi(
                PpiInterrupt::GlobalTimer,
            ))
        };
        assert!(dispatched);
        assert_eq!(CALLS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn ignores_non_matching_interrupts() {
        CALLS.store(0, Ordering::Relaxed);
        let dispatched = unsafe {
            dispatch::<typelevel::GlobalTimer, TestHandler>(Interrupt::Spi(SpiInterrupt::Usb0))
        };
        assert!(!dispatched);
        assert_eq!(CALLS.load(Ordering::Relaxed), 0);
    }
}
