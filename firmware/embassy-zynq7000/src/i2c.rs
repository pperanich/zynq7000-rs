use core::{cell::RefCell, marker::PhantomData, task::Poll};

use arbitrary_int::{u2, u6};
use critical_section::Mutex;
use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::i2c::NoAcknowledgeSource;
use zynq7000::{
    i2c::{Control, InterruptControl, InterruptStatus, MmioRegisters, TransferSize},
    slcr::reset::DualClockReset,
};

use crate::interrupt::typelevel::Interrupt as _;
use crate::{Hertz, gpio, interrupt::typelevel, pac, slcr};

pub const FIFO_DEPTH: usize = 16;
pub const MAX_READ_SIZE: usize = 255;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cId {
    I2c0 = 0,
    I2c1 = 1,
}

impl I2cId {
    const fn index(self) -> usize {
        self as usize
    }
}

pub trait Mode: sealed_mode::SealedMode {}
pub struct Blocking;
pub struct Async;

mod sealed_mode {
    pub trait SealedMode {}
}

impl sealed_mode::SealedMode for Blocking {}
impl sealed_mode::SealedMode for Async {}
impl Mode for Blocking {}
impl Mode for Async {}

#[doc(hidden)]
pub trait SealedInstance {
    fn id() -> I2cId;
    fn regs() -> pac::i2c::MmioRegisters<'static>;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + crate::PeripheralType + 'static + Send {
    type Interrupt: typelevel::Interrupt;
}

pub(crate) mod sealed {
    pub trait PinPair<T, SCL, SDA> {}
    pub trait SclPin<T> {
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait SdaPin<T> {
        fn mux_config() -> crate::gpio::MuxConfig;
    }
}

pub trait SclPin<T: Instance>: gpio::Pin + sealed::SclPin<T> {}
pub trait SdaPin<T: Instance>: gpio::Pin + sealed::SdaPin<T> {}
pub trait PinPair<T: Instance, SCL: SclPin<T>, SDA: SdaPin<T>>:
    sealed::PinPair<T, SCL, SDA>
{
}

#[derive(Debug, Clone, Copy)]
pub enum I2cSpeed {
    Normal100kHz,
    HighSpeed400KHz,
}

impl I2cSpeed {
    pub fn frequency_full_number(&self) -> Hertz {
        Hertz::from_raw(match self {
            I2cSpeed::Normal100kHz => 100_000,
            I2cSpeed::HighSpeed400KHz => 400_000,
        })
    }

    pub fn frequency_for_calculation(&self) -> Hertz {
        Hertz::from_raw(match self {
            I2cSpeed::Normal100kHz => 90_000,
            I2cSpeed::HighSpeed400KHz => 384_600,
        })
    }
}

#[derive(Debug, thiserror::Error)]
#[error("I2C speed not attainable")]
pub struct I2cSpeedNotAttainable;

#[derive(Debug, thiserror::Error, Clone, Copy, PartialEq, Eq)]
pub enum I2cTxError {
    #[error("arbitration lost")]
    ArbitrationLoss,
    #[error("transfer not acknowledged: {0}")]
    Nack(NoAcknowledgeSource),
    #[error("TX overflow")]
    TxOverflow,
    #[error("timeout of transfer")]
    Timeout,
}

#[derive(Debug, thiserror::Error, Clone, Copy, PartialEq, Eq)]
pub enum I2cRxError {
    #[error("arbitration lost")]
    ArbitrationLoss,
    #[error("transfer not acknowledged")]
    Nack(NoAcknowledgeSource),
    #[error("RX underflow")]
    RxUnderflow,
    #[error("RX overflow")]
    RxOverflow,
    #[error("timeout of transfer")]
    Timeout,
    #[error("read data exceeds maximum allowed 255 bytes per transfer")]
    ReadDataLenTooLarge,
}

#[derive(Debug, thiserror::Error, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    #[error("arbitration lost")]
    ArbitrationLoss,
    #[error("transfer not acknowledged: {0}")]
    Nack(NoAcknowledgeSource),
    #[error("TX overflow")]
    TxOverflow,
    #[error("RX underflow")]
    RxUnderflow,
    #[error("RX overflow")]
    RxOverflow,
    #[error("timeout of transfer")]
    Timeout,
    #[error("read data exceeds maximum allowed 255 bytes per transfer")]
    ReadDataLenTooLarge,
    #[error("only single read, single write, or write-read async transactions are supported")]
    UnsupportedTransaction,
}

impl From<I2cRxError> for I2cError {
    fn from(err: I2cRxError) -> Self {
        match err {
            I2cRxError::ArbitrationLoss => I2cError::ArbitrationLoss,
            I2cRxError::Nack(nack) => I2cError::Nack(nack),
            I2cRxError::RxUnderflow => I2cError::RxUnderflow,
            I2cRxError::RxOverflow => I2cError::RxOverflow,
            I2cRxError::Timeout => I2cError::Timeout,
            I2cRxError::ReadDataLenTooLarge => I2cError::ReadDataLenTooLarge,
        }
    }
}

impl From<I2cTxError> for I2cError {
    fn from(err: I2cTxError) -> Self {
        match err {
            I2cTxError::ArbitrationLoss => I2cError::ArbitrationLoss,
            I2cTxError::Nack(nack) => I2cError::Nack(nack),
            I2cTxError::TxOverflow => I2cError::TxOverflow,
            I2cTxError::Timeout => I2cError::Timeout,
        }
    }
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            I2cError::ArbitrationLoss => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            I2cError::Nack(nack_kind) => embedded_hal::i2c::ErrorKind::NoAcknowledge(*nack_kind),
            I2cError::RxOverflow => embedded_hal::i2c::ErrorKind::Overrun,
            I2cError::TxOverflow
            | I2cError::RxUnderflow
            | I2cError::Timeout
            | I2cError::ReadDataLenTooLarge
            | I2cError::UnsupportedTransaction => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

#[inline]
pub fn calculate_i2c_speed(cpu_1x_clk: Hertz, clk_config: ClockConfig) -> Hertz {
    cpu_1x_clk / (22 * (clk_config.div_a as u32 + 1) * (clk_config.div_b as u32 + 1))
}

pub fn calculate_divisors(
    cpu_1x_clk: Hertz,
    speed: I2cSpeed,
) -> Result<ClockConfig, I2cSpeedNotAttainable> {
    let target_speed = speed.frequency_for_calculation();
    if cpu_1x_clk > 22 * 64 * 4 * target_speed {
        return Err(I2cSpeedNotAttainable);
    }
    let mut smallest_deviation = u32::MAX;
    let mut best_div_a = 1;
    let mut best_div_b = 1;
    for divisor_a in 1..=4 {
        for divisor_b in 1..=64 {
            let i2c_clock = cpu_1x_clk / (22 * divisor_a * divisor_b);
            let deviation = (target_speed.raw() as i32 - i2c_clock.raw() as i32).unsigned_abs();
            if deviation < smallest_deviation {
                smallest_deviation = deviation;
                best_div_a = divisor_a;
                best_div_b = divisor_b;
            }
        }
    }
    Ok(ClockConfig::new(best_div_a as u8 - 1, best_div_b as u8 - 1))
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct ClockConfig {
    div_a: u8,
    div_b: u8,
}

impl ClockConfig {
    pub const fn new(div_a: u8, div_b: u8) -> Self {
        Self { div_a, div_b }
    }

    pub const fn div_a(&self) -> u8 {
        self.div_a
    }

    pub const fn div_b(&self) -> u8 {
        self.div_b
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub clock_config: ClockConfig,
}

impl Config {
    pub const fn new(clock_config: ClockConfig) -> Self {
        Self { clock_config }
    }
}

pub struct I2c<'d, T: Instance, M: Mode> {
    regs: MmioRegisters<'static>,
    _phantom: PhantomData<(&'d mut T, M)>,
}

impl<'d, T: Instance, M: Mode> embedded_hal::i2c::ErrorType for I2c<'d, T, M> {
    type Error = I2cError;
}

impl<'d, T: Instance> I2c<'d, T, Blocking> {
    pub fn new_blocking<SCL: SclPin<T>, SDA: SdaPin<T>>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, SCL>,
        sda: Peri<'d, SDA>,
        config: Config,
    ) -> Self
    where
        (): PinPair<T, SCL, SDA>,
    {
        configure_scl_pin::<T, SCL>(scl);
        configure_sda_pin::<T, SDA>(sda);
        Self::new_inner(config)
    }
}

impl<'d, T: Instance> I2c<'d, T, Async> {
    pub fn new_async<SCL: SclPin<T>, SDA: SdaPin<T>>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, SCL>,
        sda: Peri<'d, SDA>,
        irq: impl typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self
    where
        (): PinPair<T, SCL, SDA>,
    {
        configure_scl_pin::<T, SCL>(scl);
        configure_sda_pin::<T, SDA>(sda);
        let i2c = Self::new_inner(config);
        bind_interrupt::<T::Interrupt, InterruptHandler<T>, _>(irq);
        T::Interrupt::unpend();
        T::Interrupt::enable();
        i2c
    }
}

fn bind_interrupt<I, H, B>(_binding: B)
where
    I: typelevel::Interrupt,
    H: typelevel::Handler<I>,
    B: typelevel::Binding<I, H>,
{
    B::register();
}

impl<'d, T: Instance, M: Mode> I2c<'d, T, M> {
    fn new_inner(config: Config) -> Self {
        let id = T::id();
        let mut regs = T::regs();
        enable_i2c_clock(id);
        reset(id);
        regs.write_cr(
            Control::builder()
                .with_div_a(u2::new(config.clock_config.div_a()))
                .with_div_b(u6::new(config.clock_config.div_b()))
                .with_clear_fifo(true)
                .with_slv_mon(false)
                .with_hold_bus(false)
                .with_acken(false)
                .with_addressing(true)
                .with_mode(zynq7000::i2c::Mode::Master)
                .with_dir(zynq7000::i2c::Direction::Transmitter)
                .build(),
        );
        Self {
            regs,
            _phantom: PhantomData,
        }
    }

    unsafe fn steal() -> RawI2c {
        RawI2c { regs: T::regs() }
    }
}

impl<'d, T: Instance> embedded_hal::i2c::I2c for I2c<'d, T, Blocking> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut raw = RawI2c {
            regs: unsafe { self.regs.clone() },
        };
        let op_count = operations.len();
        for (idx, op) in operations.iter_mut().enumerate() {
            let generate_stop = idx + 1 == op_count;
            match op {
                embedded_hal::i2c::Operation::Read(items) => {
                    raw.read_transfer_blocking(address, items, generate_stop)?
                }
                embedded_hal::i2c::Operation::Write(items) => {
                    raw.write_transfer_blocking(address, items, generate_stop)?
                }
            }
        }
        Ok(())
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        let mut raw = RawI2c {
            regs: unsafe { self.regs.clone() },
        };
        raw.write_transfer_blocking(address, write, false)?;
        raw.read_transfer_blocking(address, read, true)?;
        Ok(())
    }
}

fn generate_stop_for_index(idx: usize, operation_count: usize) -> bool {
    idx + 1 == operation_count
}

impl<'d, T: Instance> embedded_hal_async::i2c::I2c for I2c<'d, T, Async> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let op_count = operations.len();
        for (idx, op) in operations.iter_mut().enumerate() {
            let generate_stop = generate_stop_for_index(idx, op_count);
            match op {
                embedded_hal_async::i2c::Operation::Read(read) => {
                    if read.len() > MAX_READ_SIZE {
                        return Err(I2cError::ReadDataLenTooLarge);
                    }
                    I2cFuture::<T>::new_read(address, read, generate_stop).await?;
                }
                embedded_hal_async::i2c::Operation::Write(write) => {
                    I2cFuture::<T>::new_write(address, write, generate_stop).await?;
                }
            }
        }
        Ok(())
    }
}

struct RawI2c {
    regs: MmioRegisters<'static>,
}

impl RawI2c {
    #[inline]
    fn start_transfer(&mut self, address: u8) {
        self.regs
            .write_addr(zynq7000::i2c::Address::new_with_raw_value(address as u32));
    }

    #[inline]
    fn clear_hold_bit(&mut self) {
        self.regs.modify_cr(|mut cr| {
            cr.set_hold_bus(false);
            cr
        });
    }

    #[inline]
    fn disable_interrupts(&mut self) {
        self.regs.write_idr(
            InterruptControl::ZERO
                .with_arbitration_lost(true)
                .with_rx_underflow(true)
                .with_tx_overflow(true)
                .with_rx_overflow(true)
                .with_slave_ready(true)
                .with_timeout(true)
                .with_nack(true)
                .with_data(true)
                .with_complete(true),
        );
    }

    #[inline]
    fn enable_tx_interrupts(&mut self) {
        self.regs.write_ier(
            InterruptControl::ZERO
                .with_arbitration_lost(true)
                .with_tx_overflow(true)
                .with_timeout(true)
                .with_nack(true)
                .with_data(true)
                .with_complete(true),
        );
    }

    #[inline]
    fn enable_rx_interrupts(&mut self) {
        self.regs.write_ier(
            InterruptControl::ZERO
                .with_arbitration_lost(true)
                .with_rx_underflow(true)
                .with_rx_overflow(true)
                .with_timeout(true)
                .with_nack(true)
                .with_data(true)
                .with_complete(true),
        );
    }

    #[inline]
    fn clear_interrupts(&mut self) {
        self.regs.modify_isr(|isr| isr);
    }

    fn configure_write_transfer(&mut self, generate_stop: bool) {
        self.regs.modify_cr(|mut cr| {
            cr.set_acken(true);
            cr.set_mode(zynq7000::i2c::Mode::Master);
            cr.set_clear_fifo(true);
            cr.set_dir(zynq7000::i2c::Direction::Transmitter);
            cr.set_hold_bus(!generate_stop);
            cr
        });
    }

    fn configure_read_transfer(&mut self, len: usize, generate_stop: bool) {
        self.regs.modify_cr(|mut cr| {
            cr.set_acken(true);
            cr.set_mode(zynq7000::i2c::Mode::Master);
            cr.set_clear_fifo(true);
            cr.set_dir(zynq7000::i2c::Direction::Receiver);
            cr.set_hold_bus(!generate_stop || len > FIFO_DEPTH || cr.hold_bus());
            cr
        });
        self.regs
            .write_transfer_size(TransferSize::new_with_raw_value(len as u32));
    }

    fn fill_tx_fifo(&mut self, data: &[u8], written: &mut usize) {
        let bytes_to_write = core::cmp::min(
            FIFO_DEPTH - self.regs.read_transfer_size().size() as usize,
            data.len().saturating_sub(*written),
        );
        (0..bytes_to_write).for_each(|_| {
            self.regs
                .write_data(zynq7000::i2c::Fifo::new_with_raw_value(
                    data[*written] as u32,
                ));
            *written += 1;
        });
    }

    fn clean_up_after_transfer_or_on_error(&mut self) {
        self.regs.modify_cr(|mut cr| {
            cr.set_acken(false);
            cr.set_hold_bus(false);
            cr.set_clear_fifo(true);
            cr
        });
    }

    fn check_and_handle_tx_errors(
        &mut self,
        isr: InterruptStatus,
        first_write_cycle: bool,
        first_chunk_len: usize,
    ) -> Result<(), I2cTxError> {
        if isr.tx_overflow() {
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cTxError::TxOverflow);
        }
        if isr.arbitration_lost() {
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cTxError::ArbitrationLoss);
        }
        if isr.nack() {
            self.clean_up_after_transfer_or_on_error();
            if first_write_cycle
                && self.regs.read_transfer_size().size() as usize + 1 == first_chunk_len
            {
                return Err(I2cTxError::Nack(NoAcknowledgeSource::Address));
            }
            return Err(I2cTxError::Nack(NoAcknowledgeSource::Data));
        }
        if isr.timeout() {
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cTxError::Timeout);
        }
        Ok(())
    }

    fn write_transfer_blocking(
        &mut self,
        addr: u8,
        data: &[u8],
        generate_stop: bool,
    ) -> Result<(), I2cTxError> {
        self.configure_write_transfer(generate_stop);
        let mut first_write_cycle = true;
        let mut addr_set = false;
        let mut written = 0;
        self.regs.modify_isr(|isr| isr);
        loop {
            let bytes_to_write = core::cmp::min(
                FIFO_DEPTH - self.regs.read_transfer_size().size() as usize,
                data.len() - written,
            );
            (0..bytes_to_write).for_each(|_| {
                self.regs
                    .write_data(zynq7000::i2c::Fifo::new_with_raw_value(
                        data[written] as u32,
                    ));
                written += 1;
            });
            if !addr_set {
                self.start_transfer(addr);
                addr_set = true;
            }
            let mut status = self.regs.read_sr();
            while status.tx_busy() {
                let isr = self.regs.read_isr();
                self.check_and_handle_tx_errors(isr, first_write_cycle, bytes_to_write)?;
                status = self.regs.read_sr();
            }
            first_write_cycle = false;
            if written == data.len() {
                break;
            }
        }
        while !self.regs.read_isr().complete() {
            let isr = self.regs.read_isr();
            self.check_and_handle_tx_errors(isr, first_write_cycle, data.len())?;
        }
        if generate_stop {
            self.clear_hold_bit();
        }
        Ok(())
    }

    fn check_and_handle_rx_errors(
        &mut self,
        read_count: usize,
        isr: InterruptStatus,
    ) -> Result<(), I2cRxError> {
        if isr.rx_overflow() {
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cRxError::RxOverflow);
        }
        if isr.rx_underflow() {
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cRxError::RxUnderflow);
        }
        if isr.nack() {
            self.clean_up_after_transfer_or_on_error();
            if read_count == 0 {
                return Err(I2cRxError::Nack(NoAcknowledgeSource::Address));
            }
            return Err(I2cRxError::Nack(NoAcknowledgeSource::Data));
        }
        if isr.timeout() {
            self.clean_up_after_transfer_or_on_error();
            return Err(I2cRxError::Timeout);
        }
        Ok(())
    }

    fn read_transfer_blocking(
        &mut self,
        addr: u8,
        data: &mut [u8],
        generate_stop: bool,
    ) -> Result<(), I2cRxError> {
        if read_completes_immediately(data.len()) {
            return Ok(());
        }
        self.configure_read_transfer(data.len(), generate_stop);
        let mut read = 0;
        if data.len() > MAX_READ_SIZE {
            return Err(I2cRxError::ReadDataLenTooLarge);
        }
        self.regs.modify_isr(|isr| isr);
        self.regs
            .write_transfer_size(TransferSize::new_with_raw_value(data.len() as u32));
        self.start_transfer(addr);
        loop {
            let mut status = self.regs.read_sr();
            loop {
                let isr = self.regs.read_isr();
                self.check_and_handle_rx_errors(read, isr)?;
                if status.rx_valid() {
                    break;
                }
                status = self.regs.read_sr();
            }
            while self.regs.read_sr().rx_valid() {
                data[read] = self.regs.read_data().data();
                read += 1;
            }
            if generate_stop && self.regs.read_transfer_size().size() as usize <= FIFO_DEPTH {
                self.clear_hold_bit();
            }
            if read == data.len() {
                break;
            }
        }
        while !self.regs.read_isr().complete() {
            let isr = self.regs.read_isr();
            self.check_and_handle_rx_errors(read, isr)?;
        }
        if generate_stop {
            self.clear_hold_bit();
            self.clean_up_after_transfer_or_on_error();
        }
        Ok(())
    }
}

static I2C_WAKERS: [AtomicWaker; 2] = [const { AtomicWaker::new() }; 2];
static I2C_CONTEXTS: [Mutex<RefCell<I2cAsyncContext>>; 2] =
    [const { Mutex::new(RefCell::new(I2cAsyncContext::new())) }; 2];
static I2C_DONE: [core::sync::atomic::AtomicBool; 2] =
    [const { core::sync::atomic::AtomicBool::new(false) }; 2];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum I2cAsyncTransfer {
    Write,
    Read,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum I2cAsyncPhase {
    Write,
    Read,
}

#[derive(Debug, Clone, Copy)]
struct I2cAsyncContext {
    transfer: Option<I2cAsyncTransfer>,
    phase: I2cAsyncPhase,
    address: u8,
    tx_ptr: usize,
    tx_len: usize,
    rx_ptr: usize,
    rx_len: usize,
    tx_progress: usize,
    rx_progress: usize,
    generate_stop: bool,
    error: Option<I2cError>,
}

impl I2cAsyncContext {
    const fn new() -> Self {
        Self {
            transfer: None,
            phase: I2cAsyncPhase::Write,
            address: 0,
            tx_ptr: 0,
            tx_len: 0,
            rx_ptr: 0,
            rx_len: 0,
            tx_progress: 0,
            rx_progress: 0,
            generate_stop: true,
            error: None,
        }
    }

    fn clear(&mut self) {
        *self = Self::new();
    }

    unsafe fn tx(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.tx_ptr as *const u8, self.tx_len) }
    }
}

fn finish_i2c_transfer(
    idx: usize,
    i2c: &mut RawI2c,
    context: &mut I2cAsyncContext,
    error: Option<I2cError>,
) {
    i2c.disable_interrupts();
    i2c.clear_interrupts();
    if context.generate_stop {
        i2c.clear_hold_bit();
        i2c.clean_up_after_transfer_or_on_error();
    }
    context.transfer = None;
    context.error = error;
    critical_section::with(|cs| {
        *I2C_CONTEXTS[idx].borrow(cs).borrow_mut() = *context;
    });
    I2C_DONE[idx].store(true, core::sync::atomic::Ordering::Relaxed);
    I2C_WAKERS[idx].wake();
}

fn async_i2c_error(
    i2c: &mut RawI2c,
    context: &I2cAsyncContext,
    isr: InterruptStatus,
) -> Option<I2cError> {
    if isr.arbitration_lost() {
        i2c.clean_up_after_transfer_or_on_error();
        return Some(I2cError::ArbitrationLoss);
    }
    if isr.timeout() {
        i2c.clean_up_after_transfer_or_on_error();
        return Some(I2cError::Timeout);
    }
    if isr.nack() {
        i2c.clean_up_after_transfer_or_on_error();
        return Some(I2cError::Nack(match context.phase {
            I2cAsyncPhase::Write => {
                if context.tx_progress == 0 {
                    NoAcknowledgeSource::Address
                } else {
                    NoAcknowledgeSource::Data
                }
            }
            I2cAsyncPhase::Read => {
                if context.rx_progress == 0 {
                    NoAcknowledgeSource::Address
                } else {
                    NoAcknowledgeSource::Data
                }
            }
        }));
    }
    match context.phase {
        I2cAsyncPhase::Write if isr.tx_overflow() => {
            i2c.clean_up_after_transfer_or_on_error();
            Some(I2cError::TxOverflow)
        }
        I2cAsyncPhase::Read if isr.rx_underflow() => {
            i2c.clean_up_after_transfer_or_on_error();
            Some(I2cError::RxUnderflow)
        }
        I2cAsyncPhase::Read if isr.rx_overflow() => {
            i2c.clean_up_after_transfer_or_on_error();
            Some(I2cError::RxOverflow)
        }
        _ => None,
    }
}

pub struct InterruptHandler<T: Instance>(PhantomData<T>);

impl<T: Instance> typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let idx = T::id().index();
        let mut i2c = unsafe { I2c::<T, Async>::steal() };
        let mut context = critical_section::with(|cs| *I2C_CONTEXTS[idx].borrow(cs).borrow());
        if context.transfer.is_none() {
            return;
        }

        let isr = i2c.regs.read_isr();
        if let Some(error) = async_i2c_error(&mut i2c, &context, isr) {
            finish_i2c_transfer(idx, &mut i2c, &mut context, Some(error));
            return;
        }

        match context.phase {
            I2cAsyncPhase::Write => {
                let tx_len = context.tx_len;
                let mut tx_progress = context.tx_progress;
                unsafe {
                    let tx = context.tx();
                    i2c.fill_tx_fifo(tx, &mut tx_progress);
                }
                context.tx_progress = tx_progress;
                if context.tx_progress == tx_len && isr.complete() {
                    finish_i2c_transfer(idx, &mut i2c, &mut context, None);
                    return;
                }
            }
            I2cAsyncPhase::Read => {
                let rx_len = context.rx_len;
                let rx_ptr = context.rx_ptr as *mut u8;
                while context.rx_progress < rx_len && i2c.regs.read_sr().rx_valid() {
                    unsafe {
                        core::ptr::write(
                            rx_ptr.add(context.rx_progress),
                            i2c.regs.read_data().data(),
                        );
                    }
                    context.rx_progress += 1;
                }
                if context.generate_stop
                    && i2c.regs.read_transfer_size().size() as usize <= FIFO_DEPTH
                {
                    i2c.clear_hold_bit();
                }
                if context.rx_progress == rx_len && isr.complete() {
                    finish_i2c_transfer(idx, &mut i2c, &mut context, None);
                    return;
                }
            }
        }

        i2c.regs.write_isr(isr);
        critical_section::with(|cs| {
            *I2C_CONTEXTS[idx].borrow(cs).borrow_mut() = context;
        });
    }
}

struct I2cFuture<T: Instance> {
    finished_regularly: bool,
    _phantom: PhantomData<T>,
}

impl<T: Instance> I2cFuture<T> {
    fn new_write(address: u8, data: &[u8], generate_stop: bool) -> Self {
        let idx = T::id().index();
        let mut i2c = unsafe { I2c::<T, Async>::steal() };
        I2C_DONE[idx].store(false, core::sync::atomic::Ordering::Relaxed);
        i2c.disable_interrupts();
        i2c.configure_write_transfer(generate_stop);
        critical_section::with(|cs| {
            let mut context = I2C_CONTEXTS[idx].borrow(cs).borrow_mut();
            context.clear();
            context.transfer = Some(I2cAsyncTransfer::Write);
            context.phase = I2cAsyncPhase::Write;
            context.address = address;
            context.generate_stop = generate_stop;
            context.tx_ptr = data.as_ptr() as usize;
            context.tx_len = data.len();
            i2c.fill_tx_fifo(data, &mut context.tx_progress);
        });
        i2c.clear_interrupts();
        i2c.enable_tx_interrupts();
        i2c.start_transfer(address);
        Self {
            finished_regularly: false,
            _phantom: PhantomData,
        }
    }

    fn new_read(address: u8, data: &mut [u8], generate_stop: bool) -> Self {
        let idx = T::id().index();
        if read_completes_immediately(data.len()) {
            critical_section::with(|cs| {
                let mut context = I2C_CONTEXTS[idx].borrow(cs).borrow_mut();
                context.clear();
                context.error = None;
            });
            I2C_DONE[idx].store(true, core::sync::atomic::Ordering::Relaxed);
            return Self {
                finished_regularly: false,
                _phantom: PhantomData,
            };
        }
        let mut i2c = unsafe { I2c::<T, Async>::steal() };
        I2C_DONE[idx].store(false, core::sync::atomic::Ordering::Relaxed);
        i2c.disable_interrupts();
        critical_section::with(|cs| {
            let mut context = I2C_CONTEXTS[idx].borrow(cs).borrow_mut();
            context.clear();
            context.transfer = Some(I2cAsyncTransfer::Read);
            context.phase = I2cAsyncPhase::Read;
            context.address = address;
            context.generate_stop = generate_stop;
            context.rx_ptr = data.as_mut_ptr() as usize;
            context.rx_len = data.len();
        });
        i2c.configure_read_transfer(data.len(), generate_stop);
        i2c.clear_interrupts();
        i2c.enable_rx_interrupts();
        i2c.start_transfer(address);
        Self {
            finished_regularly: false,
            _phantom: PhantomData,
        }
    }
}

impl<T: Instance> core::future::Future for I2cFuture<T> {
    type Output = Result<(), I2cError>;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        let this = unsafe { self.get_unchecked_mut() };
        let idx = T::id().index();
        I2C_WAKERS[idx].register(cx.waker());
        if I2C_DONE[idx].swap(false, core::sync::atomic::Ordering::Relaxed) {
            let error = critical_section::with(|cs| {
                let mut context = I2C_CONTEXTS[idx].borrow(cs).borrow_mut();
                let error = context.error;
                context.clear();
                error
            });
            this.finished_regularly = true;
            return Poll::Ready(error.map_or(Ok(()), Err));
        }
        Poll::Pending
    }
}

impl<T: Instance> Drop for I2cFuture<T> {
    fn drop(&mut self) {
        if self.finished_regularly {
            return;
        }
        let idx = T::id().index();
        let mut i2c = unsafe { I2c::<T, Async>::steal() };
        i2c.disable_interrupts();
        i2c.clear_hold_bit();
        i2c.clean_up_after_transfer_or_on_error();
        critical_section::with(|cs| {
            I2C_CONTEXTS[idx].borrow(cs).borrow_mut().clear();
        });
    }
}

#[inline]
fn read_completes_immediately(len: usize) -> bool {
    len == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stop_generation_only_releases_on_last_operation() {
        assert!(!generate_stop_for_index(0, 3));
        assert!(!generate_stop_for_index(1, 3));
        assert!(generate_stop_for_index(2, 3));
    }

    #[test]
    fn async_context_defaults_to_stop_generation() {
        assert!(I2cAsyncContext::new().generate_stop);
    }

    #[test]
    fn zero_length_blocking_read_completes_immediately() {
        assert!(read_completes_immediately(0));
        assert!(!read_completes_immediately(1));
    }
}

fn configure_scl_pin<T: Instance, P: SclPin<T>>(pin: Peri<'_, P>) {
    let pin = pin.into();
    let mut ll = gpio::LowLevelGpio::new(gpio::PinOffset::Mio(pin.offset() as usize));
    ll.configure_as_io_periph_pin(<P as sealed::SclPin<T>>::mux_config(), Some(true), None);
}

fn configure_sda_pin<T: Instance, P: SdaPin<T>>(pin: Peri<'_, P>) {
    let pin = pin.into();
    let mut ll = gpio::LowLevelGpio::new(gpio::PinOffset::Mio(pin.offset() as usize));
    ll.configure_as_io_periph_pin(<P as sealed::SdaPin<T>>::mux_config(), Some(true), None);
}

fn enable_i2c_clock(id: I2cId) {
    unsafe {
        slcr::with_unlocked(|slcr| {
            slcr.clk_ctrl().modify_aper_clk_ctrl(|mut val| {
                match id {
                    I2cId::I2c0 => val.set_i2c_0_1x_clk_act(true),
                    I2cId::I2c1 => val.set_i2c_1_1x_clk_act(true),
                }
                val
            });
        });
    }
}

#[inline]
pub fn reset(id: I2cId) {
    let assert_reset = match id {
        I2cId::I2c0 => DualClockReset::builder()
            .with_periph1_cpu1x_rst(false)
            .with_periph0_cpu1x_rst(true)
            .build(),
        I2cId::I2c1 => DualClockReset::builder()
            .with_periph1_cpu1x_rst(true)
            .with_periph0_cpu1x_rst(false)
            .build(),
    };
    unsafe {
        slcr::with_unlocked(|regs| {
            regs.reset_ctrl().write_i2c(assert_reset);
            for _ in 0..3 {
                aarch32_cpu::asm::nop();
            }
            regs.reset_ctrl().write_i2c(DualClockReset::DEFAULT);
        });
    }
}
