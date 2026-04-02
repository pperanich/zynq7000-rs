//! Type-level interrupt bindings for Embassy-aware Zynq-7000 drivers.
//!
//! Unlike HALs that install one physical IRQ entry per driver through the binding macro, Zynq
//! routes supported Embassy drivers through one crate-owned GIC entrypoint and a logical
//! dispatcher. The public contract is therefore:
//! - [`crate::bind_interrupts!`] provides the compile-time proof surface
//! - [`bind`] performs the runtime activation path (`register`, `unpend`, `enable`)
//! - raw dispatch remains internal runtime plumbing
use core::sync::atomic::{AtomicUsize, Ordering};

use arbitrary_int::traits::Integer;
use zynq7000::gic::InterruptSignalRegister;

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum PpiInterrupt {
    GlobalTimer = 27,
    NFiq = 28,
    CpuPrivateTimer = 29,
    Awdt = 30,
    NIrq = 31,
}

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum SpiInterrupt {
    Usb0 = 53,
    Eth0 = 54,
    Sdio0 = 56,
    I2c0 = 57,
    Spi0 = 58,
    Uart0 = 59,
    Ttc10 = 69,
    Usb1 = 76,
    Eth1 = 77,
    Sdio1 = 79,
    I2c1 = 80,
    Spi1 = 81,
    Uart1 = 82,
}

#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Interrupt {
    Sgi(usize),
    Ppi(PpiInterrupt),
    Spi(SpiInterrupt),
    Invalid(usize),
    Spurious,
}

type RawHandler = unsafe fn();

static HANDLERS: [AtomicUsize; 93] = [const { AtomicUsize::new(0) }; 93];

#[derive(Debug, Copy, Clone)]
pub struct InterruptInfo {
    raw_reg: InterruptSignalRegister,
    interrupt: Interrupt,
    cpu_id: u8,
}

impl InterruptInfo {
    pub(crate) fn new(raw_reg: InterruptSignalRegister) -> Self {
        let interrupt = Interrupt::from_raw(raw_reg.ack_int_id().as_u32());
        Self {
            raw_reg,
            interrupt,
            cpu_id: raw_reg.cpu_id().as_u8(),
        }
    }

    pub const fn interrupt(&self) -> Interrupt {
        self.interrupt
    }

    pub const fn cpu_id(&self) -> u8 {
        self.cpu_id
    }

    pub const fn raw_reg(&self) -> InterruptSignalRegister {
        self.raw_reg
    }
}

impl Interrupt {
    pub(crate) fn from_raw(int_id: u32) -> Interrupt {
        match int_id {
            0..=15 => Interrupt::Sgi(int_id as usize),
            27 => Interrupt::Ppi(PpiInterrupt::GlobalTimer),
            28 => Interrupt::Ppi(PpiInterrupt::NFiq),
            29 => Interrupt::Ppi(PpiInterrupt::CpuPrivateTimer),
            30 => Interrupt::Ppi(PpiInterrupt::Awdt),
            31 => Interrupt::Ppi(PpiInterrupt::NIrq),
            32..=92 => Interrupt::spi_from_raw(int_id as u8),
            1023 => Interrupt::Spurious,
            _ => Interrupt::Invalid(int_id as usize),
        }
    }

    pub(crate) fn spi_from_raw(id: u8) -> Self {
        let spi = match id {
            53 => SpiInterrupt::Usb0,
            54 => SpiInterrupt::Eth0,
            56 => SpiInterrupt::Sdio0,
            57 => SpiInterrupt::I2c0,
            58 => SpiInterrupt::Spi0,
            59 => SpiInterrupt::Uart0,
            69 => SpiInterrupt::Ttc10,
            76 => SpiInterrupt::Usb1,
            77 => SpiInterrupt::Eth1,
            79 => SpiInterrupt::Sdio1,
            80 => SpiInterrupt::I2c1,
            81 => SpiInterrupt::Spi1,
            82 => SpiInterrupt::Uart1,
            _ => return Interrupt::Invalid(id as usize),
        };
        Interrupt::Spi(spi)
    }

    pub(crate) const fn raw_id(self) -> Option<usize> {
        match self {
            Interrupt::Sgi(id) => Some(id),
            Interrupt::Ppi(ppi) => Some(ppi as usize),
            Interrupt::Spi(spi) => Some(spi as usize),
            Interrupt::Invalid(_) | Interrupt::Spurious => None,
        }
    }

    pub(crate) const fn is_supported(self) -> bool {
        matches!(self, Interrupt::Ppi(_) | Interrupt::Spi(_))
    }
}

pub mod typelevel {
    //! Type-level logical interrupt traits and bindings.

    use super::Interrupt as GicInterrupt;

    mod sealed {
        pub trait SealedInterrupt {}
    }

    /// Type-level logical interrupt.
    pub trait Interrupt: sealed::SealedInterrupt {
        /// Concrete GIC interrupt routed for this source.
        const IRQ: GicInterrupt;

        /// Enable the interrupt at the GIC.
        fn enable() {
            crate::runtime::enable_interrupt(Self::IRQ);
        }

        /// Disable the interrupt at the GIC.
        fn disable() {
            crate::runtime::disable_interrupt(Self::IRQ);
        }

        /// Clear a pending delivery at the GIC.
        fn unpend() {
            crate::runtime::unpend_interrupt(Self::IRQ);
        }
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
    pub unsafe trait Binding<I: Interrupt, H: Handler<I>>: Copy {
        /// Register the handler for the logical interrupt.
        fn register();
    }

    pub enum Usb0 {}
    impl sealed::SealedInterrupt for Usb0 {}
    impl Interrupt for Usb0 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::Usb0);
    }

    pub enum Usb1 {}
    impl sealed::SealedInterrupt for Usb1 {}
    impl Interrupt for Usb1 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::Usb1);
    }

    pub enum Uart0 {}
    impl sealed::SealedInterrupt for Uart0 {}
    impl Interrupt for Uart0 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::Uart0);
    }

    pub enum Uart1 {}
    impl sealed::SealedInterrupt for Uart1 {}
    impl Interrupt for Uart1 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::Uart1);
    }

    pub enum Spi0 {}
    impl sealed::SealedInterrupt for Spi0 {}
    impl Interrupt for Spi0 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::Spi0);
    }

    pub enum Spi1 {}
    impl sealed::SealedInterrupt for Spi1 {}
    impl Interrupt for Spi1 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::Spi1);
    }

    pub enum I2c0 {}
    impl sealed::SealedInterrupt for I2c0 {}
    impl Interrupt for I2c0 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::I2c0);
    }

    pub enum I2c1 {}
    impl sealed::SealedInterrupt for I2c1 {}
    impl Interrupt for I2c1 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::I2c1);
    }

    pub enum Sdio0 {}
    impl sealed::SealedInterrupt for Sdio0 {}
    impl Interrupt for Sdio0 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::Sdio0);
    }

    pub enum Sdio1 {}
    impl sealed::SealedInterrupt for Sdio1 {}
    impl Interrupt for Sdio1 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::Sdio1);
    }

    pub enum Eth0 {}
    impl sealed::SealedInterrupt for Eth0 {}
    impl Interrupt for Eth0 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::Eth0);
    }

    pub enum Eth1 {}
    impl sealed::SealedInterrupt for Eth1 {}
    impl Interrupt for Eth1 {
        const IRQ: GicInterrupt = GicInterrupt::Spi(super::SpiInterrupt::Eth1);
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

#[doc(hidden)]
pub fn register<I, H>()
where
    I: typelevel::Interrupt,
    H: typelevel::Handler<I>,
{
    register_internal(I::IRQ, H::on_interrupt);
}

pub(crate) fn register_internal(interrupt: Interrupt, handler_fn: RawHandler) {
    let Some(idx) = interrupt.raw_id() else {
        return;
    };
    let handler = handler_fn as *const () as usize;
    match HANDLERS[idx].compare_exchange(0, handler, Ordering::AcqRel, Ordering::Acquire) {
        Ok(_) => {}
        Err(existing) if existing == handler => {}
        Err(_) => panic!("interrupt already registered"),
    }
}

pub fn bind<I, H, B>(_binding: B)
where
    I: typelevel::Interrupt,
    H: typelevel::Handler<I>,
    B: typelevel::Binding<I, H>,
{
    B::register();
    I::unpend();
    I::enable();
}

pub(crate) fn dispatch_current() -> bool {
    crate::runtime::dispatch_interrupts(dispatch_registered)
}

pub(crate) fn dispatch_registered(interrupt: Interrupt) -> bool {
    let Some(idx) = interrupt.raw_id() else {
        return false;
    };
    let handler = HANDLERS[idx].load(Ordering::Acquire);
    if handler == 0 {
        return false;
    }
    let handler: RawHandler = unsafe { core::mem::transmute(handler) };
    unsafe { handler() };
    true
}

#[cfg(test)]
mod tests {
    use core::sync::atomic::{AtomicUsize, Ordering};

    use super::*;

    static UART0_TEST_CALLS: AtomicUsize = AtomicUsize::new(0);
    static UART1_TEST_CALLS: AtomicUsize = AtomicUsize::new(0);

    struct Uart0HandlerA;
    struct Uart0HandlerB;
    struct Uart1Handler;

    impl typelevel::Handler<typelevel::Uart0> for Uart0HandlerA {
        unsafe fn on_interrupt() {
            UART0_TEST_CALLS.fetch_add(1, Ordering::Relaxed);
        }
    }

    impl typelevel::Handler<typelevel::Uart0> for Uart0HandlerB {
        unsafe fn on_interrupt() {
            UART0_TEST_CALLS.fetch_add(100, Ordering::Relaxed);
        }
    }

    impl typelevel::Handler<typelevel::Uart1> for Uart1Handler {
        unsafe fn on_interrupt() {
            UART1_TEST_CALLS.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn reset_handler(interrupt: Interrupt) {
        if let Some(idx) = interrupt.raw_id() {
            HANDLERS[idx].store(0, Ordering::Release);
        }
    }

    #[test]
    fn interrupt_raw_mapping_round_trips_supported_ids() {
        let global_timer = Interrupt::from_raw(27);
        let uart0 = Interrupt::from_raw(59);
        let uart1 = Interrupt::from_raw(82);

        assert_eq!(global_timer, Interrupt::Ppi(PpiInterrupt::GlobalTimer));
        assert_eq!(uart0, Interrupt::Spi(SpiInterrupt::Uart0));
        assert_eq!(uart1, Interrupt::Spi(SpiInterrupt::Uart1));
        assert_eq!(global_timer.raw_id(), Some(27));
        assert_eq!(uart0.raw_id(), Some(59));
        assert_eq!(uart1.raw_id(), Some(82));
    }

    #[test]
    fn invalid_and_spurious_interrupts_are_not_dispatchable() {
        assert_eq!(Interrupt::from_raw(26), Interrupt::Invalid(26));
        assert_eq!(Interrupt::from_raw(1023), Interrupt::Spurious);
        assert_eq!(Interrupt::Invalid(200).raw_id(), None);
        assert_eq!(Interrupt::Spurious.raw_id(), None);
        assert!(!dispatch_registered(Interrupt::Invalid(200)));
        assert!(!dispatch_registered(Interrupt::Spurious));
    }

    #[test]
    fn register_allows_duplicate_identical_handler() {
        reset_handler(Interrupt::Spi(SpiInterrupt::Uart0));
        UART0_TEST_CALLS.store(0, Ordering::Relaxed);

        register::<typelevel::Uart0, Uart0HandlerA>();
        register::<typelevel::Uart0, Uart0HandlerA>();

        assert!(dispatch_registered(Interrupt::Spi(SpiInterrupt::Uart0)));
        assert_eq!(UART0_TEST_CALLS.load(Ordering::Relaxed), 1);

        reset_handler(Interrupt::Spi(SpiInterrupt::Uart0));
    }

    #[test]
    #[should_panic(expected = "interrupt already registered")]
    fn register_rejects_conflicting_handler() {
        reset_handler(Interrupt::Spi(SpiInterrupt::Uart0));
        register::<typelevel::Uart0, Uart0HandlerA>();
        register::<typelevel::Uart0, Uart0HandlerB>();
    }

    #[test]
    fn dispatch_registered_invokes_bound_handler() {
        reset_handler(Interrupt::Spi(SpiInterrupt::Uart1));
        UART1_TEST_CALLS.store(0, Ordering::Relaxed);
        register::<typelevel::Uart1, Uart1Handler>();

        assert!(dispatch_registered(Interrupt::Spi(SpiInterrupt::Uart1)));
        assert_eq!(UART1_TEST_CALLS.load(Ordering::Relaxed), 1);

        reset_handler(Interrupt::Spi(SpiInterrupt::Uart1));
    }
}
