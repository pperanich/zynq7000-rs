use core::{
    cell::RefCell,
    convert::Infallible,
    future::poll_fn,
    hint::spin_loop,
    marker::PhantomData,
    mem::ManuallyDrop,
    sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering},
    task::Poll,
};

use critical_section::Mutex;
use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;
use zynq7000::{
    slcr::reset::DualRefAndClockResetSpiUart,
    uart::{
        BaudRateDivisor, Baudgen, ChMode, ClockSelect, Fifo, FifoTrigger, InterruptControl,
        InterruptStatus, MmioRegisters, Mode,
    },
};

use crate::{Hertz, slcr};
use crate::{gpio, interrupt, interrupt::typelevel, pac};

/// FIFO depth of the UART peripheral.
pub const FIFO_DEPTH: usize = 64;
/// Default RX trigger level.
pub const DEFAULT_RX_TRIGGER_LEVEL: u8 = 1;

/// UART ID.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum UartId {
    /// UART 0.
    Uart0 = 0,
    /// UART 1.
    Uart1 = 1,
}

impl UartId {
    #[inline]
    const fn index(self) -> usize {
        self as usize
    }
}

/// Maximum acceptable baud-rate error in percentage points.
pub const MAX_BAUD_ERROR_PERCENT: f64 = 0.5;

/// Divisor zero error.
#[derive(Debug, thiserror::Error)]
#[error("divisor is zero")]
pub struct DivisorZero;

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum UartError {
    #[error("UART parity error")]
    Parity,
    #[error("UART framing error")]
    Framing,
    #[error("UART overrun error")]
    Overrun,
}

impl UartError {
    const fn into_bits(self) -> u8 {
        match self {
            Self::Parity => 1,
            Self::Framing => 2,
            Self::Overrun => 3,
        }
    }

    const fn from_bits(bits: u8) -> Option<Self> {
        match bits {
            1 => Some(Self::Parity),
            2 => Some(Self::Framing),
            3 => Some(Self::Overrun),
            _ => None,
        }
    }
}

impl embedded_io::Error for UartError {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::Parity | Self::Framing => embedded_io::ErrorKind::InvalidData,
            Self::Overrun => embedded_io::ErrorKind::OutOfMemory,
        }
    }
}

impl embedded_hal_nb::serial::Error for UartError {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        match self {
            Self::Parity => embedded_hal_nb::serial::ErrorKind::Parity,
            Self::Framing => embedded_hal_nb::serial::ErrorKind::FrameFormat,
            Self::Overrun => embedded_hal_nb::serial::ErrorKind::Overrun,
        }
    }
}

/// Parity configuration.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    /// Even parity.
    Even,
    /// Odd parity.
    Odd,
    /// No parity (default).
    #[default]
    None,
}

/// Stopbit configuration.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Stopbits {
    /// One stop bit (default).
    #[default]
    One,
    /// 1.5 stopbits.
    OnePointFive,
    /// 2 stopbits.
    Two,
}

/// Character length configuration.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum CharLen {
    /// 6 bits.
    SixBits,
    /// 7 bits.
    SevenBits,
    /// 8 bits (default).
    #[default]
    EightBits,
}

/// Clock configuration for baud rate generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    cd: u16,
    bdiv: u8,
}

impl ClockConfig {
    #[inline]
    pub const fn new(cd: u16, bdiv: u8) -> Result<Self, DivisorZero> {
        if cd == 0 {
            return Err(DivisorZero);
        }
        Ok(Self { cd, bdiv })
    }

    pub fn new_autocalc_with_error(
        io_clks: &crate::clocks::IoClocks,
        target_baud: u32,
    ) -> Result<(Self, f64), DivisorZero> {
        Self::new_autocalc_generic(io_clks, ClockSelect::UartRefClk, target_baud)
    }

    pub fn new_autocalc_generic(
        io_clks: &crate::clocks::IoClocks,
        clk_sel: ClockSelect,
        target_baud: u32,
    ) -> Result<(Self, f64), DivisorZero> {
        Self::new_autocalc_with_raw_clk(io_clks.uart_clk(), clk_sel, target_baud)
    }

    pub fn new_autocalc_with_raw_clk(
        uart_clk: Hertz,
        clk_sel: ClockSelect,
        target_baud: u32,
    ) -> Result<(Self, f64), DivisorZero> {
        calculate_raw_baud_cfg_smallest_error(uart_clk, clk_sel, target_baud)
    }

    #[inline]
    pub const fn cd(&self) -> u16 {
        self.cd
    }

    #[inline]
    pub const fn bdiv(&self) -> u8 {
        self.bdiv
    }

    #[inline]
    pub fn actual_baud(&self, sel_clk: Hertz) -> f64 {
        sel_clk.raw() as f64 / (self.cd as f64 * (self.bdiv + 1) as f64)
    }
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self::new(1, 0).unwrap()
    }
}

/// UART configuration.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    clk_config: ClockConfig,
    chmode: ChMode,
    parity: Parity,
    stopbits: Stopbits,
    chrl: CharLen,
    clk_sel: ClockSelect,
}

impl Config {
    pub fn new_with_clk_config(clk_config: ClockConfig) -> Self {
        Self::new(
            clk_config,
            ChMode::default(),
            Parity::default(),
            Stopbits::default(),
            CharLen::default(),
            ClockSelect::default(),
        )
    }

    #[inline]
    pub const fn new(
        clk_config: ClockConfig,
        chmode: ChMode,
        parity: Parity,
        stopbits: Stopbits,
        chrl: CharLen,
        clk_sel: ClockSelect,
    ) -> Self {
        Self {
            clk_config,
            chmode,
            parity,
            stopbits,
            chrl,
            clk_sel,
        }
    }

    #[inline]
    pub const fn raw_clk_config(&self) -> ClockConfig {
        self.clk_config
    }
}

pub fn calculate_raw_baud_cfg_smallest_error(
    mut uart_clk: Hertz,
    clk_sel: ClockSelect,
    target_baud: u32,
) -> Result<(ClockConfig, f64), DivisorZero> {
    if target_baud == 0 {
        return Err(DivisorZero);
    }
    if clk_sel == ClockSelect::UartRefClkDiv8 {
        uart_clk /= 8;
    }
    let mut best = ClockConfig::default();
    let mut smallest_error = 100.0;
    for bdiv in 4..u8::MAX {
        let cd =
            libm::round(uart_clk.raw() as f64 / ((bdiv as u32 + 1) as f64 * target_baud as f64))
                as u64;
        if cd == 0 || cd > u16::MAX as u64 {
            continue;
        }
        let candidate = ClockConfig {
            cd: cd as u16,
            bdiv,
        };
        let baud = candidate.actual_baud(uart_clk);
        let error = ((baud - target_baud as f64).abs() / target_baud as f64) * 100.0;
        if error < smallest_error {
            best = candidate;
            smallest_error = error;
        }
    }
    Ok((best, smallest_error))
}

#[derive(Debug, Copy, Clone)]
struct TxContext {
    ptr: usize,
    len: usize,
    progress: usize,
}

impl TxContext {
    const fn new() -> Self {
        Self {
            ptr: 0,
            len: 0,
            progress: 0,
        }
    }

    fn reset(&mut self) {
        self.ptr = 0;
        self.len = 0;
        self.progress = 0;
    }

    fn is_active(&self) -> bool {
        self.ptr != 0
    }
}

#[derive(Debug, Copy, Clone)]
struct RxContext {
    ptr: usize,
    len: usize,
    progress: usize,
}

impl RxContext {
    const fn new() -> Self {
        Self {
            ptr: 0,
            len: 0,
            progress: 0,
        }
    }

    fn reset(&mut self) {
        self.ptr = 0;
        self.len = 0;
        self.progress = 0;
    }

    fn is_active(&self) -> bool {
        self.ptr != 0
    }
}

#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
struct ClaimedPins {
    tx_mio: Option<u8>,
    rx_mio: Option<u8>,
    route_id: Option<u8>,
}

impl ClaimedPins {
    fn merge(&mut self, other: Self) {
        if let Some(tx_mio) = other.tx_mio {
            assert!(
                self.tx_mio.is_none() || self.tx_mio == Some(tx_mio),
                "UART instance already active on a different TX pin"
            );
            self.tx_mio = Some(tx_mio);
        }
        if let Some(rx_mio) = other.rx_mio {
            assert!(
                self.rx_mio.is_none() || self.rx_mio == Some(rx_mio),
                "UART instance already active on a different RX pin"
            );
            self.rx_mio = Some(rx_mio);
        }
        if let Some(route_id) = other.route_id {
            assert!(
                self.route_id.is_none() || self.route_id == Some(route_id),
                "UART instance already active on an incompatible TX/RX route group"
            );
            self.route_id = Some(route_id);
        }
    }
}

pub(crate) struct State {
    rx_waker: AtomicWaker,
    tx_waker: AtomicWaker,
    rx_done: AtomicBool,
    tx_done: AtomicBool,
    rx_ready_len: AtomicUsize,
    rx_error: AtomicU8,
    tx_flush_pending: AtomicBool,
    tx_rx_refcount: AtomicU8,
    active_config: Mutex<RefCell<Option<Config>>>,
    rx_context: Mutex<RefCell<RxContext>>,
    tx_context: Mutex<RefCell<TxContext>>,
    claimed_pins: Mutex<RefCell<ClaimedPins>>,
}

impl State {
    const fn new() -> Self {
        Self {
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
            rx_done: AtomicBool::new(false),
            tx_done: AtomicBool::new(false),
            rx_ready_len: AtomicUsize::new(0),
            rx_error: AtomicU8::new(0),
            tx_flush_pending: AtomicBool::new(false),
            tx_rx_refcount: AtomicU8::new(0),
            active_config: Mutex::new(RefCell::new(None)),
            rx_context: Mutex::new(RefCell::new(RxContext::new())),
            tx_context: Mutex::new(RefCell::new(TxContext::new())),
            claimed_pins: Mutex::new(RefCell::new(ClaimedPins {
                tx_mio: None,
                rx_mio: None,
                route_id: None,
            })),
        }
    }
}

pub(crate) static UART_STATES: [State; 2] = [const { State::new() }, const { State::new() }];

#[doc(hidden)]
pub(crate) trait SealedInstance {
    fn id() -> UartId;
    fn regs() -> pac::uart::MmioRegisters<'static>;
    fn state() -> &'static State;
}

/// UART peripheral instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    type Interrupt: typelevel::Interrupt;
}

#[allow(private_interfaces)]
pub(crate) mod sealed {
    pub trait RouteGroup {}

    pub trait Same<T> {}
    impl<T> Same<T> for T {}

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum PinDirection {
        Tx,
        Rx,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PinMetadata {
        pub mio: u8,
        pub direction: PinDirection,
        pub route_id: u8,
        pub mux_config: crate::gpio::MuxConfig,
    }

    pub trait TxPin<T> {
        type RouteGroup: RouteGroup;
        fn metadata() -> PinMetadata;
    }
    pub trait RxPin<T> {
        type RouteGroup: RouteGroup;
        fn metadata() -> PinMetadata;
    }
}

/// TX pin for a UART instance.
pub trait TxPin<T: Instance>: gpio::Pin + sealed::TxPin<T> {}

/// RX pin for a UART instance.
pub trait RxPin<T: Instance>: gpio::Pin + sealed::RxPin<T> {}

/// Embassy UART interrupt handler.
pub struct InterruptHandler<T: Instance>(PhantomData<T>);

impl<T: Instance> typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::regs();
        let state = T::state();
        on_interrupt_tx(T::id(), unsafe { regs.clone() }, state);
        on_interrupt_rx(T::id(), regs, state);
    }
}

/// Native UART driver.
pub struct Uart<'d, T: Instance> {
    tx: UartTx<'d, T>,
    rx: UartRx<'d, T>,
}

/// UART TX half.
pub struct UartTx<'d, T: Instance> {
    regs: MmioRegisters<'static>,
    id: UartId,
    _lifetime: PhantomData<&'d mut T>,
}

/// UART RX half.
pub struct UartRx<'d, T: Instance> {
    regs: MmioRegisters<'static>,
    id: UartId,
    _lifetime: PhantomData<&'d mut T>,
}

/// Type-erased TX half used by the blocking logger.
pub struct AnyTx {
    regs: MmioRegisters<'static>,
    id: UartId,
}

impl<'d, T: Instance> Uart<'d, T> {
    pub fn new_blocking<TX, RX>(
        _uart: Peri<'d, T>,
        tx: Peri<'d, TX>,
        rx: Peri<'d, RX>,
        config: Config,
    ) -> Self
    where
        TX: TxPin<T>,
        RX: RxPin<T>,
        <TX as sealed::TxPin<T>>::RouteGroup: sealed::Same<<RX as sealed::RxPin<T>>::RouteGroup>,
    {
        configure_mio_pin_for_uart_tx::<T, _>(tx);
        configure_mio_pin_for_uart_rx::<T, _>(rx);
        let regs = acquire_uart(
            T::id(),
            T::regs(),
            config,
            2,
            ClaimedPins {
                tx_mio: Some(<TX as sealed::TxPin<T>>::metadata().mio),
                rx_mio: Some(<RX as sealed::RxPin<T>>::metadata().mio),
                route_id: Some(<TX as sealed::TxPin<T>>::metadata().route_id),
            },
        );
        Self {
            tx: UartTx {
                regs: unsafe { regs.clone() },
                id: T::id(),
                _lifetime: PhantomData,
            },
            rx: UartRx {
                regs,
                id: T::id(),
                _lifetime: PhantomData,
            },
        }
    }

    pub fn new<TX, RX>(
        uart: Peri<'d, T>,
        tx: Peri<'d, TX>,
        rx: Peri<'d, RX>,
        irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self
    where
        TX: TxPin<T>,
        RX: RxPin<T>,
        <TX as sealed::TxPin<T>>::RouteGroup: sealed::Same<<RX as sealed::RxPin<T>>::RouteGroup>,
    {
        let mut uart = Self::new_blocking(uart, tx, rx, config);
        bind_interrupt::<T::Interrupt, InterruptHandler<T>, _>(irq);
        uart.tx.clear_interrupts();
        uart.tx.disable_interrupts();
        uart
    }

    pub fn split(self) -> (UartTx<'d, T>, UartRx<'d, T>) {
        (self.tx, self.rx)
    }

    pub fn split_by_ref(&mut self) -> (&mut UartTx<'d, T>, &mut UartRx<'d, T>) {
        (&mut self.tx, &mut self.rx)
    }

    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, UartError> {
        self.rx.read(buf).await
    }
}

impl<'d, T: Instance> UartTx<'d, T> {
    pub fn new_blocking<TX>(_uart: Peri<'d, T>, tx: Peri<'d, TX>, config: Config) -> Self
    where
        TX: TxPin<T>,
    {
        let metadata = <TX as sealed::TxPin<T>>::metadata();
        configure_mio_pin_for_uart(tx, metadata);
        let regs = acquire_uart(
            T::id(),
            T::regs(),
            config,
            1,
            ClaimedPins {
                tx_mio: Some(metadata.mio),
                rx_mio: None,
                route_id: Some(metadata.route_id),
            },
        );
        Self {
            regs,
            id: T::id(),
            _lifetime: PhantomData,
        }
    }

    pub fn new<TX>(
        uart: Peri<'d, T>,
        tx: Peri<'d, TX>,
        irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self
    where
        TX: TxPin<T>,
    {
        let mut tx = Self::new_blocking(uart, tx, config);
        tx.clear_interrupts();
        tx.disable_interrupts();
        bind_interrupt::<T::Interrupt, InterruptHandler<T>, _>(irq);
        tx
    }

    pub fn into_any(self) -> AnyTx {
        let this = ManuallyDrop::new(self);
        AnyTx {
            regs: unsafe { this.regs.clone() },
            id: this.id,
        }
    }

    pub async fn write(&mut self, buf: &[u8]) -> usize {
        if buf.is_empty() {
            return 0;
        }

        let state = state_for_id(self.id);
        let mut guard = TxTransferDropGuard::new(self.id, unsafe { self.regs.clone() });
        state.tx_done.store(false, Ordering::Relaxed);
        self.disable_interrupts();
        self.disable();

        let init_fill_count = core::cmp::min(buf.len(), FIFO_DEPTH);
        critical_section::with(|cs| {
            let context_ref = state.tx_context.borrow(cs);
            let mut context = context_ref.borrow_mut();
            context.ptr = buf.as_ptr() as usize;
            context.len = buf.len();
            context.progress = init_fill_count;
        });

        self.enable(true);
        for byte in buf.iter().take(init_fill_count) {
            self.write_fifo_unchecked(*byte);
        }
        self.enable_interrupts();

        poll_fn(|cx| {
            T::state().tx_waker.register(cx.waker());
            if state.tx_done.swap(false, Ordering::Relaxed) {
                guard.defuse();
                Poll::Ready(buf.len())
            } else {
                Poll::Pending
            }
        })
        .await
    }

    #[inline]
    pub fn enable(&mut self, with_reset: bool) {
        if with_reset {
            self.soft_reset();
        }
        self.regs.modify_cr(|mut val| {
            val.set_tx_en(true);
            val.set_tx_dis(false);
            val
        });
    }

    #[inline]
    pub fn disable(&mut self) {
        self.regs.modify_cr(|mut val| {
            val.set_tx_en(false);
            val.set_tx_dis(true);
            val
        });
    }

    #[inline]
    pub fn soft_reset(&mut self) {
        self.regs.modify_cr(|mut val| {
            val.set_tx_rst(true);
            val
        });
        while self.regs.read_cr().tx_rst() {}
    }

    #[inline]
    pub fn write_fifo_unchecked(&mut self, word: u8) {
        self.regs.write_fifo(Fifo::new_with_raw_value(word as u32));
    }

    #[inline]
    pub fn write_fifo(&mut self, word: u8) -> nb::Result<(), Infallible> {
        if self.regs.read_sr().tx_full() {
            return Err(nb::Error::WouldBlock);
        }
        self.write_fifo_unchecked(word);
        Ok(())
    }

    pub fn flush_blocking(&mut self) {
        while !self.regs.read_sr().tx_empty() {}
    }

    #[inline]
    pub fn enable_interrupts(&mut self) {
        self.regs.write_ier(tx_interrupt_control());
    }

    #[inline]
    pub fn disable_interrupts(&mut self) {
        self.regs.write_idr(tx_interrupt_control());
    }

    #[inline]
    pub fn clear_interrupts(&mut self) {
        self.regs.write_isr(tx_interrupt_status());
    }
}

fn state_for_id(id: UartId) -> &'static State {
    &UART_STATES[id.index()]
}

fn acquire_uart(
    id: UartId,
    regs: MmioRegisters<'static>,
    config: Config,
    parts: u8,
    claimed_pins: ClaimedPins,
) -> MmioRegisters<'static> {
    let state = state_for_id(id);
    let should_init = critical_section::with(|cs| {
        let refcount = state.tx_rx_refcount.load(Ordering::Relaxed);
        let config_ref = state.active_config.borrow(cs);
        let mut active_config = config_ref.borrow_mut();
        let claimed_pins_ref = state.claimed_pins.borrow(cs);
        let mut active_pins = claimed_pins_ref.borrow_mut();

        if refcount == 0 {
            state.tx_rx_refcount.store(parts, Ordering::Relaxed);
            active_config.replace(config);
            active_pins.merge(claimed_pins);
            true
        } else {
            assert_eq!(
                active_config
                    .as_ref()
                    .map(|active| uart_configs_match(*active, config)),
                Some(true),
                "UART instance already active with a different configuration"
            );
            active_pins.merge(claimed_pins);
            state
                .tx_rx_refcount
                .store(refcount.saturating_add(parts), Ordering::Relaxed);
            false
        }
    });

    if should_init {
        init_uart(id, regs, config)
    } else {
        regs
    }
}

fn rx_interrupt_should_complete(ready: usize, has_timeout: bool, error: Option<UartError>) -> bool {
    error.is_some() || ready != 0 || (has_timeout && ready != 0)
}

fn uart_configs_match(left: Config, right: Config) -> bool {
    left.raw_clk_config() == right.raw_clk_config()
        && core::mem::discriminant(&left.chmode) == core::mem::discriminant(&right.chmode)
        && left.parity == right.parity
        && left.stopbits == right.stopbits
        && left.chrl == right.chrl
        && core::mem::discriminant(&left.clk_sel) == core::mem::discriminant(&right.clk_sel)
}

fn bind_interrupt<I, H, B>(_binding: B)
where
    I: typelevel::Interrupt,
    H: typelevel::Handler<I>,
    B: typelevel::Binding<I, H>,
{
    B::register();
    I::unpend();
    I::enable();
}

struct TxTransferDropGuard {
    id: UartId,
    regs: MmioRegisters<'static>,
    active: bool,
}

impl TxTransferDropGuard {
    fn new(id: UartId, regs: MmioRegisters<'static>) -> Self {
        Self {
            id,
            regs,
            active: true,
        }
    }

    fn defuse(&mut self) {
        self.reset_context();
        self.active = false;
    }

    fn reset_context(&mut self) {
        let state = state_for_id(self.id);
        self.regs.write_idr(tx_interrupt_control());
        self.regs.write_isr(tx_interrupt_status());
        state.tx_done.store(false, Ordering::Relaxed);
        state.tx_flush_pending.store(false, Ordering::Relaxed);
        critical_section::with(|cs| {
            let context_ref = state.tx_context.borrow(cs);
            context_ref.borrow_mut().reset();
        });
    }
}

impl Drop for TxTransferDropGuard {
    fn drop(&mut self) {
        if self.active {
            self.reset_context();
        }
    }
}

struct TxFlushDropGuard {
    id: UartId,
    regs: MmioRegisters<'static>,
    active: bool,
}

impl TxFlushDropGuard {
    fn new(id: UartId, regs: MmioRegisters<'static>) -> Self {
        Self {
            id,
            regs,
            active: true,
        }
    }

    fn defuse(&mut self) {
        self.reset_context();
        self.active = false;
    }

    fn reset_context(&mut self) {
        let state = state_for_id(self.id);
        self.regs.write_idr(tx_interrupt_control());
        self.regs.write_isr(tx_interrupt_status());
        state.tx_done.store(false, Ordering::Relaxed);
        state.tx_flush_pending.store(false, Ordering::Relaxed);
    }
}

impl Drop for TxFlushDropGuard {
    fn drop(&mut self) {
        if self.active {
            self.reset_context();
        }
    }
}

struct RxTransferDropGuard {
    id: UartId,
    regs: MmioRegisters<'static>,
    active: bool,
}

impl RxTransferDropGuard {
    fn new(id: UartId, regs: MmioRegisters<'static>) -> Self {
        Self {
            id,
            regs,
            active: true,
        }
    }

    fn defuse(&mut self) {
        self.reset_context();
        self.active = false;
    }

    fn reset_context(&mut self) {
        let state = state_for_id(self.id);
        self.regs.write_idr(rx_interrupt_control());
        self.regs.write_isr(rx_interrupt_status());
        state.rx_done.store(false, Ordering::Relaxed);
        state.rx_ready_len.store(0, Ordering::Relaxed);
        state.rx_error.store(0, Ordering::Relaxed);
        critical_section::with(|cs| {
            let context_ref = state.rx_context.borrow(cs);
            context_ref.borrow_mut().reset();
        });
    }
}

impl Drop for RxTransferDropGuard {
    fn drop(&mut self) {
        if self.active {
            self.reset_context();
        }
    }
}

impl<'d, T: Instance> UartRx<'d, T> {
    pub fn new_blocking<RX>(_uart: Peri<'d, T>, rx: Peri<'d, RX>, config: Config) -> Self
    where
        RX: RxPin<T>,
    {
        let metadata = <RX as sealed::RxPin<T>>::metadata();
        configure_mio_pin_for_uart(rx, metadata);
        let regs = acquire_uart(
            T::id(),
            T::regs(),
            config,
            1,
            ClaimedPins {
                tx_mio: None,
                rx_mio: Some(metadata.mio),
                route_id: Some(metadata.route_id),
            },
        );
        Self {
            regs,
            id: T::id(),
            _lifetime: PhantomData,
        }
    }

    pub fn new<RX>(
        uart: Peri<'d, T>,
        rx: Peri<'d, RX>,
        irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self
    where
        RX: RxPin<T>,
    {
        let mut rx = Self::new_blocking(uart, rx, config);
        rx.clear_interrupts();
        rx.disable_interrupts();
        bind_interrupt::<T::Interrupt, InterruptHandler<T>, _>(irq);
        rx
    }

    #[inline]
    pub fn read_fifo(&mut self) -> nb::Result<u8, UartError> {
        if let Some(error) = take_rx_error(&mut self.regs) {
            return Err(nb::Error::Other(error));
        }
        if self.regs.read_sr().rx_empty() {
            return Err(nb::Error::WouldBlock);
        }
        Ok(self.regs.read_fifo().fifo())
    }

    #[inline]
    pub fn read_fifo_unchecked(&mut self) -> u8 {
        self.regs.read_fifo().fifo()
    }

    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, UartError> {
        if buf.is_empty() {
            return Ok(0);
        }

        let state = state_for_id(self.id);
        let mut guard = RxTransferDropGuard::new(self.id, unsafe { self.regs.clone() });
        state.rx_done.store(false, Ordering::Relaxed);
        state.rx_ready_len.store(0, Ordering::Relaxed);
        state.rx_error.store(0, Ordering::Relaxed);
        self.disable_interrupts();
        self.clear_interrupts();

        critical_section::with(|cs| {
            let context_ref = state.rx_context.borrow(cs);
            let mut context = context_ref.borrow_mut();
            context.ptr = buf.as_mut_ptr() as usize;
            context.len = buf.len();
            context.progress = 0;
        });

        if let Some(error) = take_rx_error(&mut self.regs) {
            guard.defuse();
            return Err(error);
        }
        let ready = service_rx_fifo(&mut self.regs, state);
        if ready != 0 {
            if let Some(error) = take_rx_error(&mut self.regs) {
                guard.defuse();
                return Err(error);
            }
            guard.defuse();
            return Ok(ready);
        }

        self.enable_interrupts();

        poll_fn(|cx| {
            state.rx_waker.register(cx.waker());
            if state.rx_done.swap(false, Ordering::Relaxed) {
                guard.defuse();
                let error = state.rx_error.swap(0, Ordering::Relaxed);
                if let Some(error) = UartError::from_bits(error) {
                    Poll::Ready(Err(error))
                } else {
                    Poll::Ready(Ok(state.rx_ready_len.swap(0, Ordering::Relaxed)))
                }
            } else {
                Poll::Pending
            }
        })
        .await
    }

    #[inline]
    pub fn enable_interrupts(&mut self) {
        self.regs.write_ier(rx_interrupt_control());
    }

    #[inline]
    pub fn disable_interrupts(&mut self) {
        self.regs.write_idr(rx_interrupt_control());
    }

    #[inline]
    pub fn clear_interrupts(&mut self) {
        self.regs.write_isr(rx_interrupt_status());
    }
}

impl AnyTx {
    fn write_fifo(&mut self, word: u8) -> nb::Result<(), Infallible> {
        if self.regs.read_sr().tx_full() {
            return Err(nb::Error::WouldBlock);
        }
        self.regs.write_fifo(Fifo::new_with_raw_value(word as u32));
        Ok(())
    }

    fn flush_blocking(&mut self) {
        while !self.regs.read_sr().tx_empty() {}
    }
}

fn configure_mio_pin_for_uart_tx<T: Instance, P: TxPin<T>>(pin: Peri<'_, P>) {
    configure_mio_pin_for_uart(pin, <P as sealed::TxPin<T>>::metadata());
}

fn configure_mio_pin_for_uart_rx<T: Instance, P: RxPin<T>>(pin: Peri<'_, P>) {
    configure_mio_pin_for_uart(pin, <P as sealed::RxPin<T>>::metadata());
}

fn configure_mio_pin_for_uart<T: gpio::Pin>(pin: Peri<'_, T>, metadata: sealed::PinMetadata) {
    let pin = pin.into();
    debug_assert_eq!(
        pin.offset(),
        metadata.mio,
        "UART pin metadata does not match the token-backed MIO pin"
    );
    let mut ll = gpio::LowLevelGpio::new(gpio::PinOffset::Mio(pin.offset() as usize));
    ll.configure_as_io_periph_pin(metadata.mux_config, None, None);
}

fn init_uart(id: UartId, mut regs: MmioRegisters<'static>, cfg: Config) -> MmioRegisters<'static> {
    enable_uart_clock(id);
    reset_uart(id);

    regs.modify_cr(|mut v| {
        v.set_tx_dis(true);
        v.set_rx_dis(true);
        v
    });
    regs.write_idr(InterruptControl::new_with_raw_value(0xFFFF_FFFF));

    let mode = Mode::builder()
        .with_chmode(cfg.chmode)
        .with_nbstop(match cfg.stopbits {
            Stopbits::One => zynq7000::uart::Stopbits::One,
            Stopbits::OnePointFive => zynq7000::uart::Stopbits::OnePointFive,
            Stopbits::Two => zynq7000::uart::Stopbits::Two,
        })
        .with_par(match cfg.parity {
            Parity::Even => zynq7000::uart::Parity::Even,
            Parity::Odd => zynq7000::uart::Parity::Odd,
            Parity::None => zynq7000::uart::Parity::NoParity,
        })
        .with_chrl(match cfg.chrl {
            CharLen::SixBits => zynq7000::uart::CharLen::SixBits,
            CharLen::SevenBits => zynq7000::uart::CharLen::SevenBits,
            CharLen::EightBits => zynq7000::uart::CharLen::EightBits,
        })
        .with_clksel(cfg.clk_sel)
        .build();
    regs.write_mr(mode);
    regs.write_baudgen(
        Baudgen::builder()
            .with_cd(cfg.raw_clk_config().cd())
            .build(),
    );
    regs.write_baud_rate_div(
        BaudRateDivisor::builder()
            .with_bdiv(cfg.raw_clk_config().bdiv())
            .build(),
    );
    regs.modify_cr(|mut v| {
        v.set_tx_rst(true);
        v.set_rx_rst(true);
        v
    });

    regs.write_rx_fifo_trigger(FifoTrigger::new_with_raw_value(
        DEFAULT_RX_TRIGGER_LEVEL as u32,
    ));

    regs.modify_cr(|mut v| {
        v.set_tx_dis(false);
        v.set_rx_dis(false);
        v.set_tx_en(true);
        v.set_rx_en(true);
        v
    });

    regs
}

fn enable_uart_clock(id: UartId) {
    unsafe {
        slcr::with_unlocked(|slcr| {
            slcr.clk_ctrl().modify_aper_clk_ctrl(|mut val| {
                match id {
                    UartId::Uart0 => val.set_uart_0_1x_clk_act(true),
                    UartId::Uart1 => val.set_uart_1_1x_clk_act(true),
                }
                val
            });
            slcr.clk_ctrl().modify_uart_clk_ctrl(|mut val| {
                match id {
                    UartId::Uart0 => val.set_clk_0_act(true),
                    UartId::Uart1 => val.set_clk_1_act(true),
                }
                val
            });
        });
    }
}

fn reset_uart(id: UartId) {
    let assert_reset = match id {
        UartId::Uart0 => DualRefAndClockResetSpiUart::builder()
            .with_periph1_ref_rst(false)
            .with_periph0_ref_rst(true)
            .with_periph1_cpu1x_rst(false)
            .with_periph0_cpu1x_rst(true)
            .build(),
        UartId::Uart1 => DualRefAndClockResetSpiUart::builder()
            .with_periph1_ref_rst(true)
            .with_periph0_ref_rst(false)
            .with_periph1_cpu1x_rst(true)
            .with_periph0_cpu1x_rst(false)
            .build(),
    };
    unsafe {
        slcr::with_unlocked(|regs| {
            regs.reset_ctrl().write_uart(assert_reset);
            for _ in 0..5 {
                spin_loop();
            }
            regs.reset_ctrl()
                .write_uart(DualRefAndClockResetSpiUart::ZERO);
        });
    }
}

fn tx_interrupt_control() -> InterruptControl {
    InterruptControl::builder()
        .with_tx_over(true)
        .with_tx_near_full(true)
        .with_tx_trig(false)
        .with_rx_dms(false)
        .with_rx_timeout(false)
        .with_rx_parity(false)
        .with_rx_framing(false)
        .with_rx_over(false)
        .with_tx_full(true)
        .with_tx_empty(true)
        .with_rx_full(false)
        .with_rx_empty(false)
        .with_rx_trg(false)
        .build()
}

fn tx_interrupt_status() -> InterruptStatus {
    InterruptStatus::builder()
        .with_tx_over(true)
        .with_tx_near_full(true)
        .with_tx_trig(false)
        .with_rx_dms(false)
        .with_rx_timeout(false)
        .with_rx_parity(false)
        .with_rx_framing(false)
        .with_rx_over(false)
        .with_tx_full(true)
        .with_tx_empty(true)
        .with_rx_full(false)
        .with_rx_empty(false)
        .with_rx_trg(false)
        .build()
}

fn rx_interrupt_control() -> InterruptControl {
    InterruptControl::builder()
        .with_tx_over(false)
        .with_tx_near_full(false)
        .with_tx_trig(false)
        .with_rx_dms(false)
        .with_rx_timeout(true)
        .with_rx_parity(true)
        .with_rx_framing(true)
        .with_rx_over(true)
        .with_tx_full(false)
        .with_tx_empty(false)
        .with_rx_full(true)
        .with_rx_empty(false)
        .with_rx_trg(true)
        .build()
}

fn rx_interrupt_status() -> InterruptStatus {
    InterruptStatus::builder()
        .with_tx_over(false)
        .with_tx_near_full(false)
        .with_tx_trig(false)
        .with_rx_dms(false)
        .with_rx_timeout(true)
        .with_rx_parity(true)
        .with_rx_framing(true)
        .with_rx_over(true)
        .with_tx_full(false)
        .with_tx_empty(false)
        .with_rx_full(true)
        .with_rx_empty(false)
        .with_rx_trg(true)
        .build()
}

fn on_interrupt_tx(_id: UartId, mut regs: MmioRegisters<'static>, state: &'static State) {
    let imr = regs.read_imr();
    if !imr.tx_over() && !imr.tx_near_full() && !imr.tx_full() && !imr.tx_empty() {
        return;
    }

    let isr = regs.read_isr();
    let done = critical_section::with(|cs| {
        let context_ref = state.tx_context.borrow(cs);
        let mut context = context_ref.borrow_mut();
        if !context.is_active() {
            return state.tx_flush_pending.load(Ordering::Relaxed) && isr.tx_empty();
        }
        let remaining = context.len.saturating_sub(context.progress);
        if remaining == 0 && isr.tx_empty() {
            return true;
        }

        let slice = unsafe { core::slice::from_raw_parts(context.ptr as *const u8, context.len) };
        while context.progress < context.len {
            if regs.read_sr().tx_full() {
                break;
            }
            regs.write_fifo(Fifo::new_with_raw_value(slice[context.progress] as u32));
            context.progress += 1;
        }
        context.progress >= context.len && regs.read_sr().tx_empty()
    });

    regs.write_isr(tx_interrupt_status());
    if done {
        regs.write_idr(tx_interrupt_control());
        state.tx_done.store(true, Ordering::Relaxed);
        state.tx_flush_pending.store(false, Ordering::Relaxed);
        state.tx_waker.wake();
    }
}

fn on_interrupt_rx(_id: UartId, mut regs: MmioRegisters<'static>, state: &'static State) {
    let imr = regs.read_imr();
    if !imr.rx_full()
        && !imr.rx_trg()
        && !imr.rx_timeout()
        && !imr.rx_parity()
        && !imr.rx_framing()
        && !imr.rx_over()
    {
        return;
    }

    let isr = regs.read_isr();
    let has_timeout = isr.rx_timeout();
    let error = if isr.rx_parity() {
        Some(UartError::Parity)
    } else if isr.rx_framing() {
        Some(UartError::Framing)
    } else if isr.rx_over() {
        Some(UartError::Overrun)
    } else {
        None
    };
    if !isr.rx_full() && !isr.rx_trg() && !has_timeout && error.is_none() {
        return;
    }

    let ready = service_rx_fifo(&mut regs, state);
    regs.write_isr(rx_interrupt_status());
    if rx_interrupt_should_complete(ready, has_timeout, error) {
        regs.write_idr(rx_interrupt_control());
        if let Some(error) = error {
            state.rx_error.store(error.into_bits(), Ordering::Relaxed);
        }
        state.rx_ready_len.store(ready, Ordering::Relaxed);
        state.rx_done.store(true, Ordering::Relaxed);
        state.rx_waker.wake();
    }
}

fn service_rx_fifo(regs: &mut MmioRegisters<'static>, state: &'static State) -> usize {
    critical_section::with(|cs| {
        let context_ref = state.rx_context.borrow(cs);
        let mut context = context_ref.borrow_mut();
        if !context.is_active() {
            return 0;
        }

        let slice = unsafe { core::slice::from_raw_parts_mut(context.ptr as *mut u8, context.len) };
        while context.progress < context.len {
            if regs.read_sr().rx_empty() {
                break;
            }
            slice[context.progress] = regs.read_fifo().fifo();
            context.progress += 1;
        }
        context.progress
    })
}

fn take_rx_error(regs: &mut MmioRegisters<'static>) -> Option<UartError> {
    let isr = regs.read_isr();
    let error = if isr.rx_parity() {
        Some(UartError::Parity)
    } else if isr.rx_framing() {
        Some(UartError::Framing)
    } else if isr.rx_over() {
        Some(UartError::Overrun)
    } else {
        None
    };
    if error.is_some() {
        regs.write_isr(rx_interrupt_status());
    }
    error
}

fn drop_tx_rx(id: UartId, mut regs: MmioRegisters<'static>, state: &'static State) {
    if !drop_releases_uart(state.tx_rx_refcount.fetch_sub(1, Ordering::Relaxed)) {
        return;
    }

    regs.write_idr(InterruptControl::new_with_raw_value(0xFFFF_FFFF));
    regs.write_isr(InterruptStatus::new_with_raw_value(0xFFFF_FFFF));
    state.tx_done.store(false, Ordering::Relaxed);
    state.rx_done.store(false, Ordering::Relaxed);
    state.rx_ready_len.store(0, Ordering::Relaxed);
    state.rx_error.store(0, Ordering::Relaxed);
    state.tx_flush_pending.store(false, Ordering::Relaxed);
    let claimed_pins = critical_section::with(|cs| {
        let claimed = *state.claimed_pins.borrow(cs).borrow();
        *state.claimed_pins.borrow(cs).borrow_mut() = ClaimedPins {
            tx_mio: None,
            rx_mio: None,
            route_id: None,
        };
        state.active_config.borrow(cs).borrow_mut().take();
        state.tx_context.borrow(cs).borrow_mut().reset();
        state.rx_context.borrow(cs).borrow_mut().reset();
        claimed
    });
    if let Some(mio) = claimed_pins.tx_mio {
        gpio::LowLevelGpio::new(gpio::PinOffset::Mio(mio as usize)).configure_as_disconnected();
    }
    if let Some(mio) = claimed_pins.rx_mio {
        gpio::LowLevelGpio::new(gpio::PinOffset::Mio(mio as usize)).configure_as_disconnected();
    }
    reset_uart(id);
}

#[inline]
fn drop_releases_uart(previous_refcount: u8) -> bool {
    previous_refcount == 1
}

impl<'d, T: Instance> embedded_hal_nb::serial::ErrorType for Uart<'d, T> {
    type Error = UartError;
}

impl<'d, T: Instance> embedded_hal_nb::serial::Write for Uart<'d, T> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        match self.tx.write_fifo(word) {
            Ok(()) => Ok(()),
            Err(nb::Error::WouldBlock) => Err(nb::Error::WouldBlock),
            Err(nb::Error::Other(err)) => match err {},
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        if self.tx.regs.read_sr().tx_empty() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<'d, T: Instance> embedded_hal_nb::serial::Read for Uart<'d, T> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx.read_fifo()
    }
}

impl<'d, T: Instance> embedded_io::ErrorType for Uart<'d, T> {
    type Error = UartError;
}

impl<'d, T: Instance> embedded_io::Write for Uart<'d, T> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        match embedded_io::Write::write(&mut self.tx, buf) {
            Ok(written) => Ok(written),
            Err(err) => match err {},
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        match embedded_io::Write::flush(&mut self.tx) {
            Ok(()) => Ok(()),
            Err(err) => match err {},
        }
    }
}

impl<'d, T: Instance> embedded_io::Read for Uart<'d, T> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io::Read::read(&mut self.rx, buf)
    }
}

impl<'d, T: Instance> embedded_io::ErrorType for UartTx<'d, T> {
    type Error = Infallible;
}

impl<'d, T: Instance> embedded_io::Write for UartTx<'d, T> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        while self.regs.read_sr().tx_full() {}
        let mut written = 0;
        for byte in buf {
            match self.write_fifo(*byte) {
                Ok(()) => written += 1,
                Err(nb::Error::WouldBlock) => return Ok(written),
                Err(nb::Error::Other(_)) => unreachable!(),
            }
        }
        Ok(written)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_blocking();
        Ok(())
    }
}

impl embedded_io::ErrorType for AnyTx {
    type Error = Infallible;
}

impl embedded_io::Write for AnyTx {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        while self.regs.read_sr().tx_full() {}
        let mut written = 0;
        for byte in buf {
            match self.write_fifo(*byte) {
                Ok(()) => written += 1,
                Err(nb::Error::WouldBlock) => return Ok(written),
                Err(nb::Error::Other(_)) => unreachable!(),
            }
        }
        Ok(written)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_blocking();
        Ok(())
    }
}

impl<'d, T: Instance> embedded_io::ErrorType for UartRx<'d, T> {
    type Error = UartError;
}

impl<'d, T: Instance> embedded_io::Read for UartRx<'d, T> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut read = 0;
        for slot in buf {
            match self.read_fifo() {
                Ok(byte) => {
                    *slot = byte;
                    read += 1;
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(err)) => return Err(err),
            }
        }
        Ok(read)
    }
}

impl<'d, T: Instance> Drop for UartTx<'d, T> {
    fn drop(&mut self) {
        drop_tx_rx(self.id, unsafe { self.regs.clone() }, state_for_id(self.id));
    }
}

impl<'d, T: Instance> Drop for UartRx<'d, T> {
    fn drop(&mut self) {
        drop_tx_rx(self.id, unsafe { self.regs.clone() }, state_for_id(self.id));
    }
}

impl<'d, T: Instance> embedded_io_async::Write for Uart<'d, T> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Result::<usize, Self::Error>::Ok(self.tx.write(buf).await)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush().await;
        Result::<(), Self::Error>::Ok(())
    }
}

impl<'d, T: Instance> embedded_io_async::Write for UartTx<'d, T> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Result::<usize, Self::Error>::Ok(UartTx::write(self, buf).await)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        if self.regs.read_sr().tx_empty() {
            return Ok(());
        }

        let state = state_for_id(self.id);
        let mut guard = TxFlushDropGuard::new(self.id, unsafe { self.regs.clone() });
        state.tx_done.store(false, Ordering::Relaxed);
        state.tx_flush_pending.store(true, Ordering::Relaxed);
        self.clear_interrupts();
        self.enable_interrupts();

        poll_fn(|cx| {
            state.tx_waker.register(cx.waker());
            if state.tx_done.swap(false, Ordering::Relaxed) {
                guard.defuse();
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
        Result::<(), Self::Error>::Ok(())
    }
}

impl<'d, T: Instance> embedded_io_async::Read for Uart<'d, T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf).await
    }
}

impl<'d, T: Instance> embedded_io_async::Read for UartRx<'d, T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        UartRx::read(self, buf).await
    }
}

impl<'d, T: Instance> From<UartTx<'d, T>> for AnyTx {
    fn from(value: UartTx<'d, T>) -> Self {
        value.into_any()
    }
}

impl Drop for AnyTx {
    fn drop(&mut self) {
        drop_tx_rx(self.id, unsafe { self.regs.clone() }, state_for_id(self.id));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rx_timeout_without_data_does_not_complete() {
        assert!(!rx_interrupt_should_complete(0, true, None));
    }

    #[test]
    fn rx_timeout_with_data_completes() {
        assert!(rx_interrupt_should_complete(1, true, None));
    }

    #[test]
    fn rx_error_completes_even_without_data() {
        assert!(rx_interrupt_should_complete(
            0,
            false,
            Some(UartError::Overrun)
        ));
    }

    #[test]
    fn drop_releases_uart_only_for_last_reference() {
        assert!(drop_releases_uart(1));
        assert!(!drop_releases_uart(2));
    }
}
