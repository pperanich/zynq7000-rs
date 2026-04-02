use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

use arbitrary_int::{prelude::*, u2, u3, u6};
use embassy_futures::yield_now;
use embassy_hal_internal::Peri;
pub use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3, Mode as SpiMode};
use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};
pub use zynq7000::qspi::LinearQspiConfig;
use zynq7000::{
    SpiClockPhase, SpiClockPolarity,
    qspi::{
        BaudRateDivisor, Config as QspiRegistersConfig, InstructionCode, InterruptStatus,
        LoopbackMasterClockDelay, SpiEnable,
    },
    slcr::{
        clocks::{SingleCommonPeriphIoClockControl, SrcSelIo},
        mio::IoType,
        reset::ResetControlQspiSmc,
    },
};

use crate::{Hertz, gpio, pac, slcr};

#[path = "qspi_lqspi_configs.rs"]
mod lqspi_configs;

pub const FIFO_DEPTH: usize = 63;
pub const MAX_BYTES_PER_TRANSFER_IO_MODE: usize = FIFO_DEPTH * 4;
pub const QSPI_START_ADDRESS: usize = 0xFC00_0000;
const COMMAND_WAIT_LIMIT: usize = 1_000_000;

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
    fn regs() -> pac::qspi::MmioRegisters<'static>;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + crate::PeripheralType + 'static + Send {}

pub(crate) mod sealed {
    pub trait ChipSelect0Pin<T> {
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait ChipSelect1Pin<T> {
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait Io0Pin<T> {
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait Io1Pin<T> {
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait Io2Pin<T> {
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait Io3Pin<T> {
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait ClockPin<T> {
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait FeedbackClockPin<T> {
        fn mux_config() -> crate::gpio::MuxConfig;
    }
}

pub trait ChipSelect0Pin<T: Instance>: gpio::Pin + sealed::ChipSelect0Pin<T> {}
pub trait ChipSelect1Pin<T: Instance>: gpio::Pin + sealed::ChipSelect1Pin<T> {}
pub trait Io0Pin<T: Instance>: gpio::Pin + sealed::Io0Pin<T> {}
pub trait Io1Pin<T: Instance>: gpio::Pin + sealed::Io1Pin<T> {}
pub trait Io2Pin<T: Instance>: gpio::Pin + sealed::Io2Pin<T> {}
pub trait Io3Pin<T: Instance>: gpio::Pin + sealed::Io3Pin<T> {}
pub trait ClockPin<T: Instance>: gpio::Pin + sealed::ClockPin<T> {}
pub trait FeedbackClockPin<T: Instance>: gpio::Pin + sealed::FeedbackClockPin<T> {}

#[derive(Debug, thiserror::Error)]
pub enum ClockCalculationError {
    #[error("target QSPI reference clock must be non-zero")]
    ZeroTargetClock,
    #[error("reference divisor out of range")]
    RefDivOutOfRange,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BaudRateConfig {
    WithLoopback,
    WithoutLoopback(BaudRateDivisor),
}

impl BaudRateConfig {
    #[inline]
    pub const fn baud_rate_divisor(&self) -> BaudRateDivisor {
        match self {
            BaudRateConfig::WithLoopback => BaudRateDivisor::_2,
            BaudRateConfig::WithoutLoopback(divisor) => *divisor,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    pub src_sel: SrcSelIo,
    pub ref_clk_div: u6,
    pub baud_rate_config: BaudRateConfig,
}

impl ClockConfig {
    pub const fn new(src_sel: SrcSelIo, ref_clk_div: u6, baud_rate_config: BaudRateConfig) -> Self {
        Self {
            src_sel,
            ref_clk_div,
            baud_rate_config,
        }
    }

    pub fn calculate_with_ref_clk(
        ref_clk: Hertz,
        src_sel: SrcSelIo,
        target_qspi_ref_clock: Hertz,
        baud_rate_config: BaudRateConfig,
    ) -> Result<Self, ClockCalculationError> {
        if target_qspi_ref_clock.raw() == 0 {
            return Err(ClockCalculationError::ZeroTargetClock);
        }
        let ref_clk_div = ref_clk.raw().div_ceil(target_qspi_ref_clock.raw());
        if ref_clk_div > u6::MAX.as_u32() {
            return Err(ClockCalculationError::RefDivOutOfRange);
        }
        Ok(Self {
            src_sel,
            ref_clk_div: u6::new(ref_clk_div as u8),
            baud_rate_config,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressWidth {
    Bits24,
    Bits32,
}

impl AddressWidth {
    const fn command_words(self) -> usize {
        match self {
            AddressWidth::Bits24 => 1,
            AddressWidth::Bits32 => 2,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QspiVendor {
    WinbondAndSpansion,
    Micron,
}

pub type OperatingMode = InstructionCode;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QspiDeviceCombination {
    pub vendor: QspiVendor,
    pub operating_mode: OperatingMode,
    pub two_devices: bool,
}

impl From<QspiDeviceCombination> for LinearQspiConfig {
    fn from(value: QspiDeviceCombination) -> Self {
        linear_mode_config_for_common_devices(value)
    }
}

pub const fn linear_mode_config_for_common_devices(
    dev_combination: QspiDeviceCombination,
) -> LinearQspiConfig {
    match dev_combination.operating_mode {
        InstructionCode::Read => {
            if dev_combination.two_devices {
                lqspi_configs::RD_TWO
            } else {
                lqspi_configs::RD_ONE
            }
        }
        InstructionCode::FastRead => {
            if dev_combination.two_devices {
                lqspi_configs::FAST_RD_TWO
            } else {
                lqspi_configs::FAST_RD_ONE
            }
        }
        InstructionCode::FastReadDualOutput => {
            if dev_combination.two_devices {
                lqspi_configs::DUAL_OUT_FAST_RD_TWO
            } else {
                lqspi_configs::DUAL_OUT_FAST_RD_ONE
            }
        }
        InstructionCode::FastReadQuadOutput => {
            if dev_combination.two_devices {
                lqspi_configs::QUAD_OUT_FAST_RD_TWO
            } else {
                lqspi_configs::QUAD_OUT_FAST_RD_ONE
            }
        }
        InstructionCode::FastReadDualIo => {
            match (dev_combination.vendor, dev_combination.two_devices) {
                (QspiVendor::WinbondAndSpansion, false) => {
                    lqspi_configs::winbond_spansion::DUAL_IO_FAST_RD_ONE
                }
                (QspiVendor::WinbondAndSpansion, true) => {
                    lqspi_configs::winbond_spansion::DUAL_IO_FAST_RD_TWO
                }
                (QspiVendor::Micron, false) => lqspi_configs::micron::DUAL_IO_FAST_RD_ONE,
                (QspiVendor::Micron, true) => lqspi_configs::micron::DUAL_IO_FAST_RD_TWO,
            }
        }
        InstructionCode::FastReadQuadIo => {
            match (dev_combination.vendor, dev_combination.two_devices) {
                (QspiVendor::WinbondAndSpansion, false) => {
                    lqspi_configs::winbond_spansion::QUAD_IO_FAST_RD_ONE
                }
                (QspiVendor::WinbondAndSpansion, true) => {
                    lqspi_configs::winbond_spansion::QUAD_IO_FAST_RD_TWO
                }
                (QspiVendor::Micron, false) => lqspi_configs::micron::QUAD_IO_FAST_RD_ONE,
                (QspiVendor::Micron, true) => lqspi_configs::micron::QUAD_IO_FAST_RD_TWO,
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub clock_config: ClockConfig,
    pub mode: SpiMode,
    pub voltage: IoType,
    pub linear_config: LinearQspiConfig,
    pub page_size: usize,
    pub capacity: usize,
    pub erase_opcode: u8,
    pub erase_size: u32,
    pub read_status_opcode: u8,
    pub write_enable_opcode: u8,
    pub clear_status_opcode: Option<u8>,
    pub page_program_opcode: u8,
    pub address_width: AddressWidth,
}

impl Config {
    pub const fn spansion_s25fl256s(
        clock_config: ClockConfig,
        voltage: IoType,
        linear_config: LinearQspiConfig,
    ) -> Self {
        Self {
            clock_config,
            mode: MODE_0,
            voltage,
            linear_config,
            page_size: 256,
            capacity: 32 * 1024 * 1024,
            erase_opcode: 0xD8,
            erase_size: 0x10000,
            read_status_opcode: 0x05,
            write_enable_opcode: 0x06,
            clear_status_opcode: Some(0x30),
            page_program_opcode: 0x02,
            address_width: AddressWidth::Bits32,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ControllerMode {
    Linear,
    Io,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("offset out of range")]
    OutOfRange,
    #[error("erase size does not match the driver configuration")]
    InvalidEraseSize,
    #[error("write size must be exactly 4 bytes")]
    InvalidWriteSize,
    #[error("write is not aligned to the required flash write size")]
    UnalignedWrite,
    #[error("erase is not aligned to the required erase size")]
    UnalignedErase,
    #[error("write crosses a flash page boundary")]
    PageBoundary,
    #[error("write enable did not latch")]
    WriteEnableFailed,
    #[error("device remained busy for too long")]
    BusyTimeout,
    #[error("command response timed out")]
    CommandTimeout,
    #[error("operation failed according to device status")]
    OperationFailed,
}

#[inline]
fn validate_write_size<const WRITE_SIZE: usize>() -> Result<(), Error> {
    if WRITE_SIZE == 4 {
        Ok(())
    } else {
        Err(Error::InvalidWriteSize)
    }
}

impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Error::OutOfRange
            | Error::InvalidEraseSize
            | Error::InvalidWriteSize
            | Error::UnalignedWrite
            | Error::UnalignedErase
            | Error::PageBoundary => NorFlashErrorKind::OutOfBounds,
            Error::WriteEnableFailed
            | Error::BusyTimeout
            | Error::CommandTimeout
            | Error::OperationFailed => NorFlashErrorKind::Other,
        }
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

struct QspiLowLevel(pac::qspi::MmioRegisters<'static>);

impl QspiLowLevel {
    fn new(regs: pac::qspi::MmioRegisters<'static>) -> Self {
        Self(regs)
    }

    fn regs(&mut self) -> &mut pac::qspi::MmioRegisters<'static> {
        &mut self.0
    }

    fn initialize(&mut self, clock_config: ClockConfig, mode: SpiMode) {
        enable_qspi_clock();
        reset();
        let (cpol, cpha) = spi_mode_const_to_cpol_cpha(mode);
        unsafe {
            slcr::with_unlocked(|slcr| {
                slcr.clk_ctrl().write_lqspi_clk_ctrl(
                    SingleCommonPeriphIoClockControl::builder()
                        .with_divisor(clock_config.ref_clk_div)
                        .with_srcsel(clock_config.src_sel)
                        .with_clk_act(true)
                        .build(),
                );
            });
        }
        self.0.write_config(
            QspiRegistersConfig::builder()
                .with_interface_mode(zynq7000::qspi::InterfaceMode::FlashMemoryInterface)
                .with_edianness(zynq7000::qspi::Endianness::Little)
                .with_holdb_dr(true)
                .with_manual_start_command(false)
                .with_manual_start_enable(false)
                .with_manual_cs(false)
                .with_peripheral_chip_select(false)
                .with_fifo_width(u2::new(0b11))
                .with_baud_rate_div(clock_config.baud_rate_config.baud_rate_divisor())
                .with_clock_phase(cpha)
                .with_clock_polarity(cpol)
                .with_mode_select(true)
                .build(),
        );
        if clock_config.baud_rate_config == BaudRateConfig::WithLoopback {
            self.0.write_loopback_master_clock_delay(
                LoopbackMasterClockDelay::builder()
                    .with_use_loopback(true)
                    .with_delay_1(u2::new(0))
                    .with_delay_0(u3::new(0))
                    .build(),
            );
        }
    }

    fn enable_linear_addressing(&mut self, config: LinearQspiConfig) {
        self.0
            .write_spi_enable(SpiEnable::builder().with_enable(false).build());
        self.0.modify_config(|mut val| {
            val.set_manual_start_enable(false);
            val.set_manual_cs(false);
            val.set_peripheral_chip_select(false);
            val
        });
        self.0.write_linear_qspi_config(config);
    }

    fn enable_io_mode(&mut self, dual_flash: bool) {
        self.0.modify_config(|mut val| {
            val.set_manual_start_enable(true);
            val.set_manual_cs(true);
            val
        });
        self.0.write_rx_fifo_threshold(0x1);
        self.0.write_tx_fifo_threshold(0x1);
        self.0.write_linear_qspi_config(
            LinearQspiConfig::builder()
                .with_enable_linear_mode(false)
                .with_both_memories(dual_flash)
                .with_separate_memory_bus(dual_flash)
                .with_upper_memory_page(false)
                .with_mode_enable(false)
                .with_mode_on(true)
                .with_mode_bits(0xA0)
                .with_num_dummy_bytes(u3::new(0x2))
                .with_instruction_code(InstructionCode::FastReadQuadIo)
                .build(),
        );
    }
}

pub struct Qspi<
    'd,
    T: Instance,
    M: Mode,
    const WRITE_SIZE: usize = 4,
    const ERASE_SIZE: usize = 0x10000,
> {
    ll: QspiLowLevel,
    config: Config,
    controller_mode: ControllerMode,
    _phantom: PhantomData<(&'d mut T, M)>,
}

impl<'d, T: Instance, M: Mode, const WRITE_SIZE: usize, const ERASE_SIZE: usize> ErrorType
    for Qspi<'d, T, M, WRITE_SIZE, ERASE_SIZE>
{
    type Error = Error;
}

impl<'d, T: Instance, const WRITE_SIZE: usize, const ERASE_SIZE: usize>
    Qspi<'d, T, Blocking, WRITE_SIZE, ERASE_SIZE>
{
    pub fn new_blocking<
        CS: ChipSelect0Pin<T>,
        IO0: Io0Pin<T>,
        IO1: Io1Pin<T>,
        IO2: Io2Pin<T>,
        IO3: Io3Pin<T>,
        CLK: ClockPin<T>,
    >(
        _peri: Peri<'d, T>,
        cs: Peri<'d, CS>,
        io0: Peri<'d, IO0>,
        io1: Peri<'d, IO1>,
        io2: Peri<'d, IO2>,
        io3: Peri<'d, IO3>,
        clk: Peri<'d, CLK>,
        config: Config,
    ) -> Result<Self, Error> {
        validate_write_size::<WRITE_SIZE>()?;
        if config.erase_size as usize != ERASE_SIZE || config.erase_size == 0 {
            return Err(Error::InvalidEraseSize);
        }
        configure_qspi_pin(
            cs,
            <CS as sealed::ChipSelect0Pin<T>>::mux_config(),
            Some(true),
            config.voltage,
        );
        configure_qspi_pin(
            io0,
            <IO0 as sealed::Io0Pin<T>>::mux_config(),
            Some(false),
            config.voltage,
        );
        configure_qspi_pin(
            io1,
            <IO1 as sealed::Io1Pin<T>>::mux_config(),
            Some(false),
            config.voltage,
        );
        configure_qspi_pin(
            io2,
            <IO2 as sealed::Io2Pin<T>>::mux_config(),
            Some(false),
            config.voltage,
        );
        configure_qspi_pin(
            io3,
            <IO3 as sealed::Io3Pin<T>>::mux_config(),
            Some(false),
            config.voltage,
        );
        configure_qspi_pin(
            clk,
            <CLK as sealed::ClockPin<T>>::mux_config(),
            Some(false),
            config.voltage,
        );

        let mut ll = QspiLowLevel::new(T::regs());
        ll.initialize(config.clock_config, config.mode);
        ll.enable_linear_addressing(config.linear_config);
        Ok(Self {
            ll,
            config,
            controller_mode: ControllerMode::Linear,
            _phantom: PhantomData,
        })
    }
}

impl<'d, T: Instance, const WRITE_SIZE: usize, const ERASE_SIZE: usize>
    Qspi<'d, T, Async, WRITE_SIZE, ERASE_SIZE>
{
    pub fn new_async<
        CS: ChipSelect0Pin<T>,
        IO0: Io0Pin<T>,
        IO1: Io1Pin<T>,
        IO2: Io2Pin<T>,
        IO3: Io3Pin<T>,
        CLK: ClockPin<T>,
    >(
        peri: Peri<'d, T>,
        cs: Peri<'d, CS>,
        io0: Peri<'d, IO0>,
        io1: Peri<'d, IO1>,
        io2: Peri<'d, IO2>,
        io3: Peri<'d, IO3>,
        clk: Peri<'d, CLK>,
        config: Config,
    ) -> Result<Self, Error> {
        Qspi::<T, Blocking, WRITE_SIZE, ERASE_SIZE>::new_blocking(
            peri, cs, io0, io1, io2, io3, clk, config,
        )
        .map(|qspi| qspi.into_async())
    }
}

impl<'d, T: Instance, M: Mode, const WRITE_SIZE: usize, const ERASE_SIZE: usize>
    Qspi<'d, T, M, WRITE_SIZE, ERASE_SIZE>
{
    fn into_async(self) -> Qspi<'d, T, Async, WRITE_SIZE, ERASE_SIZE> {
        Qspi {
            ll: self.ll,
            config: self.config,
            controller_mode: self.controller_mode,
            _phantom: PhantomData,
        }
    }

    fn ensure_linear_mode(&mut self) {
        if self.controller_mode != ControllerMode::Linear {
            self.ll.enable_linear_addressing(self.config.linear_config);
            self.controller_mode = ControllerMode::Linear;
        }
    }

    fn ensure_io_mode(&mut self) {
        if self.controller_mode != ControllerMode::Io {
            self.ll
                .enable_io_mode(self.config.linear_config.both_memories());
            self.controller_mode = ControllerMode::Io;
        }
    }

    fn write_command_address(
        address_width: AddressWidth,
        transfer: &mut QspiIoTransferGuard<'_>,
        opcode: u8,
        address: u32,
    ) -> usize {
        match address_width {
            AddressWidth::Bits24 => {
                transfer.write_word_txd_00(u32::from_ne_bytes([
                    opcode,
                    ((address >> 16) & 0xff) as u8,
                    ((address >> 8) & 0xff) as u8,
                    (address & 0xff) as u8,
                ]));
                AddressWidth::Bits24.command_words()
            }
            AddressWidth::Bits32 => {
                transfer.write_word_txd_01(opcode as u32);
                transfer.write_word_txd_00(u32::from_ne_bytes(address.to_be_bytes()));
                AddressWidth::Bits32.command_words()
            }
        }
    }

    fn wait_for_rx_words_blocking(
        transfer: &mut QspiIoTransferGuard<'_>,
        target_words: usize,
    ) -> Result<u32, Error> {
        let mut last_word = 0;
        let mut spins = 0;
        let mut words = 0;
        while words < target_words {
            if transfer.read_status().rx_above_threshold() {
                last_word = transfer.read_rx_data();
                words += 1;
                spins = 0;
            } else {
                spins += 1;
                if spins >= COMMAND_WAIT_LIMIT {
                    return Err(Error::CommandTimeout);
                }
            }
        }
        Ok(last_word)
    }

    async fn wait_for_rx_words_async(
        transfer: &mut QspiIoTransferGuard<'_>,
        target_words: usize,
    ) -> Result<u32, Error> {
        let mut last_word = 0;
        let mut spins = 0;
        let mut words = 0;
        while words < target_words {
            if transfer.read_status().rx_above_threshold() {
                last_word = transfer.read_rx_data();
                words += 1;
                spins = 0;
            } else {
                spins += 1;
                if spins >= COMMAND_WAIT_LIMIT {
                    return Err(Error::CommandTimeout);
                }
                yield_now().await;
            }
        }
        Ok(last_word)
    }

    fn queue_program_words(
        transfer: &mut QspiIoTransferGuard<'_>,
        bytes: &[u8],
        queued_words: &mut usize,
        total_words: usize,
    ) {
        while *queued_words < total_words && !transfer.read_status().tx_full() {
            let byte_index = *queued_words * 4;
            transfer.write_word_txd_00(u32::from_ne_bytes(
                bytes[byte_index..byte_index + 4].try_into().unwrap(),
            ));
            *queued_words += 1;
        }
    }

    fn drain_program_blocking(
        transfer: &mut QspiIoTransferGuard<'_>,
        bytes: &[u8],
        header_words: usize,
    ) -> Result<(), Error> {
        let total_data_words = bytes.len() / 4;
        let total_rx_words = header_words + total_data_words;
        let mut queued_words = 0;
        let mut rx_words = 0;
        let mut spins = 0;
        Self::queue_program_words(transfer, bytes, &mut queued_words, total_data_words);
        transfer.start();
        while rx_words < total_rx_words {
            if transfer.read_status().rx_above_threshold() {
                transfer.read_rx_data();
                rx_words += 1;
                Self::queue_program_words(transfer, bytes, &mut queued_words, total_data_words);
                spins = 0;
            } else {
                Self::queue_program_words(transfer, bytes, &mut queued_words, total_data_words);
                spins += 1;
                if spins >= COMMAND_WAIT_LIMIT {
                    return Err(Error::CommandTimeout);
                }
            }
        }
        Ok(())
    }

    async fn drain_program_async(
        transfer: &mut QspiIoTransferGuard<'_>,
        bytes: &[u8],
        header_words: usize,
    ) -> Result<(), Error> {
        let total_data_words = bytes.len() / 4;
        let total_rx_words = header_words + total_data_words;
        let mut queued_words = 0;
        let mut rx_words = 0;
        let mut spins = 0;
        Self::queue_program_words(transfer, bytes, &mut queued_words, total_data_words);
        transfer.start();
        while rx_words < total_rx_words {
            if transfer.read_status().rx_above_threshold() {
                transfer.read_rx_data();
                rx_words += 1;
                Self::queue_program_words(transfer, bytes, &mut queued_words, total_data_words);
                spins = 0;
            } else {
                Self::queue_program_words(transfer, bytes, &mut queued_words, total_data_words);
                spins += 1;
                if spins >= COMMAND_WAIT_LIMIT {
                    return Err(Error::CommandTimeout);
                }
                yield_now().await;
            }
        }
        Ok(())
    }

    pub fn capacity(&self) -> usize {
        self.config.capacity
    }

    pub fn blocking_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        if offset as usize + bytes.len() > self.config.capacity {
            return Err(Error::OutOfRange);
        }
        self.ensure_linear_mode();
        self.ll
            .0
            .write_spi_enable(SpiEnable::builder().with_enable(true).build());
        unsafe {
            core::ptr::copy_nonoverlapping(
                (QSPI_START_ADDRESS + offset as usize) as *const u8,
                bytes.as_mut_ptr(),
                bytes.len(),
            );
        }
        self.ll
            .0
            .write_spi_enable(SpiEnable::builder().with_enable(false).build());
        Ok(())
    }

    pub fn read_status(&mut self) -> Result<u8, Error> {
        self.ensure_io_mode();
        let mut transfer = QspiIoTransferGuard::new(&mut self.ll);
        transfer.write_word_txd_10(self.config.read_status_opcode as u32);
        transfer.start();
        let reply = Self::wait_for_rx_words_blocking(&mut transfer, 1)?;
        drop(transfer);
        self.ensure_linear_mode();
        Ok(((reply >> 24) & 0xff) as u8)
    }

    pub fn write_enable(&mut self) -> Result<(), Error> {
        self.ensure_io_mode();
        let mut transfer = QspiIoTransferGuard::new(&mut self.ll);
        transfer.write_word_txd_01(self.config.write_enable_opcode as u32);
        transfer.start();
        Self::wait_for_rx_words_blocking(&mut transfer, 1)?;
        drop(transfer);
        self.ensure_linear_mode();
        let status = self.read_status()?;
        if (status & 0x02) == 0 {
            return Err(Error::WriteEnableFailed);
        }
        Ok(())
    }

    fn wait_ready(&mut self) -> Result<(), Error> {
        for _ in 0..1_000_000 {
            if (self.read_status()? & 0x01) == 0 {
                return Ok(());
            }
        }
        Err(Error::BusyTimeout)
    }

    pub fn blocking_erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        let erase_size = self.config.erase_size;
        if from > to || to as usize > self.config.capacity {
            return Err(Error::OutOfRange);
        }
        if from % erase_size != 0 || to % erase_size != 0 {
            return Err(Error::UnalignedErase);
        }
        let mut address = from;
        while address < to {
            self.write_enable()?;
            self.ensure_io_mode();
            let mut transfer = QspiIoTransferGuard::new(&mut self.ll);
            let command_words = Self::write_command_address(
                self.config.address_width,
                &mut transfer,
                self.config.erase_opcode,
                address,
            );
            transfer.start();
            Self::wait_for_rx_words_blocking(&mut transfer, command_words)?;
            drop(transfer);
            self.ensure_linear_mode();
            self.wait_ready()?;
            self.check_operation_status()?;
            address += erase_size;
        }
        Ok(())
    }

    pub fn blocking_write(&mut self, mut offset: u32, mut bytes: &[u8]) -> Result<(), Error> {
        if offset as usize + bytes.len() > self.config.capacity {
            return Err(Error::OutOfRange);
        }
        if offset as usize % WRITE_SIZE != 0 || bytes.len() % WRITE_SIZE != 0 {
            return Err(Error::UnalignedWrite);
        }
        while !bytes.is_empty() {
            let page_offset = offset as usize % self.config.page_size;
            let chunk_len = core::cmp::min(self.config.page_size - page_offset, bytes.len());
            if page_offset + chunk_len > self.config.page_size {
                return Err(Error::PageBoundary);
            }
            self.write_enable()?;
            self.ensure_io_mode();
            let mut transfer = QspiIoTransferGuard::new(&mut self.ll);
            let header_words = Self::write_command_address(
                self.config.address_width,
                &mut transfer,
                self.config.page_program_opcode,
                offset,
            );
            Self::drain_program_blocking(&mut transfer, &bytes[..chunk_len], header_words)?;
            drop(transfer);
            self.ensure_linear_mode();
            self.wait_ready()?;
            self.check_operation_status()?;
            offset += chunk_len as u32;
            bytes = &bytes[chunk_len..];
        }
        Ok(())
    }

    async fn read_status_async(&mut self) -> Result<u8, Error> {
        self.ensure_io_mode();
        let mut transfer = QspiIoTransferGuard::new(&mut self.ll);
        transfer.write_word_txd_10(self.config.read_status_opcode as u32);
        transfer.start();
        let reply = Self::wait_for_rx_words_async(&mut transfer, 1).await?;
        drop(transfer);
        self.ensure_linear_mode();
        Ok(((reply >> 24) & 0xff) as u8)
    }

    async fn write_enable_async(&mut self) -> Result<(), Error> {
        self.ensure_io_mode();
        let mut transfer = QspiIoTransferGuard::new(&mut self.ll);
        transfer.write_word_txd_01(self.config.write_enable_opcode as u32);
        transfer.start();
        Self::wait_for_rx_words_async(&mut transfer, 1).await?;
        drop(transfer);
        self.ensure_linear_mode();
        if (self.read_status_async().await? & 0x02) == 0 {
            return Err(Error::WriteEnableFailed);
        }
        Ok(())
    }

    async fn wait_ready_async(&mut self) -> Result<(), Error> {
        for _ in 0..COMMAND_WAIT_LIMIT {
            if (self.read_status_async().await? & 0x01) == 0 {
                return Ok(());
            }
            yield_now().await;
        }
        Err(Error::BusyTimeout)
    }

    async fn erase_async(&mut self, from: u32, to: u32) -> Result<(), Error> {
        let erase_size = self.config.erase_size;
        if from > to || to as usize > self.config.capacity {
            return Err(Error::OutOfRange);
        }
        if from % erase_size != 0 || to % erase_size != 0 {
            return Err(Error::UnalignedErase);
        }
        let mut address = from;
        while address < to {
            self.write_enable_async().await?;
            self.ensure_io_mode();
            let mut transfer = QspiIoTransferGuard::new(&mut self.ll);
            let command_words = Self::write_command_address(
                self.config.address_width,
                &mut transfer,
                self.config.erase_opcode,
                address,
            );
            transfer.start();
            Self::wait_for_rx_words_async(&mut transfer, command_words).await?;
            drop(transfer);
            self.ensure_linear_mode();
            self.wait_ready_async().await?;
            self.check_operation_status_async().await?;
            address += erase_size;
        }
        Ok(())
    }

    async fn write_async(&mut self, mut offset: u32, mut bytes: &[u8]) -> Result<(), Error> {
        if offset as usize + bytes.len() > self.config.capacity {
            return Err(Error::OutOfRange);
        }
        if offset as usize % WRITE_SIZE != 0 || bytes.len() % WRITE_SIZE != 0 {
            return Err(Error::UnalignedWrite);
        }
        while !bytes.is_empty() {
            let page_offset = offset as usize % self.config.page_size;
            let chunk_len = core::cmp::min(self.config.page_size - page_offset, bytes.len());
            if page_offset + chunk_len > self.config.page_size {
                return Err(Error::PageBoundary);
            }
            self.write_enable_async().await?;
            self.ensure_io_mode();
            let mut transfer = QspiIoTransferGuard::new(&mut self.ll);
            let header_words = Self::write_command_address(
                self.config.address_width,
                &mut transfer,
                self.config.page_program_opcode,
                offset,
            );
            Self::drain_program_async(&mut transfer, &bytes[..chunk_len], header_words).await?;
            drop(transfer);
            self.ensure_linear_mode();
            self.wait_ready_async().await?;
            self.check_operation_status_async().await?;
            offset += chunk_len as u32;
            bytes = &bytes[chunk_len..];
        }
        Ok(())
    }

    fn check_operation_status(&mut self) -> Result<(), Error> {
        if let Some(clear_status) = self.config.clear_status_opcode {
            if (self.read_status()? & 0x60) != 0 {
                self.ensure_io_mode();
                let mut transfer = QspiIoTransferGuard::new(&mut self.ll);
                transfer.write_word_txd_01(clear_status as u32);
                transfer.start();
                Self::wait_for_rx_words_blocking(&mut transfer, 1)?;
                drop(transfer);
                self.ensure_linear_mode();
                return Err(Error::OperationFailed);
            }
        }
        Ok(())
    }

    async fn check_operation_status_async(&mut self) -> Result<(), Error> {
        if let Some(clear_status) = self.config.clear_status_opcode {
            if (self.read_status_async().await? & 0x60) != 0 {
                self.ensure_io_mode();
                let mut transfer = QspiIoTransferGuard::new(&mut self.ll);
                transfer.write_word_txd_01(clear_status as u32);
                transfer.start();
                Self::wait_for_rx_words_async(&mut transfer, 1).await?;
                drop(transfer);
                self.ensure_linear_mode();
                return Err(Error::OperationFailed);
            }
        }
        Ok(())
    }
}

impl<'d, T: Instance, M: Mode, const WRITE_SIZE: usize, const ERASE_SIZE: usize> ReadNorFlash
    for Qspi<'d, T, M, WRITE_SIZE, ERASE_SIZE>
{
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.capacity()
    }
}

impl<'d, T: Instance, M: Mode, const WRITE_SIZE: usize, const ERASE_SIZE: usize> NorFlash
    for Qspi<'d, T, M, WRITE_SIZE, ERASE_SIZE>
{
    const WRITE_SIZE: usize = WRITE_SIZE;
    const ERASE_SIZE: usize = ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.blocking_erase(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(offset, bytes)
    }
}

impl<'d, T: Instance, const WRITE_SIZE: usize, const ERASE_SIZE: usize>
    embedded_storage_async::nor_flash::ReadNorFlash for Qspi<'d, T, Async, WRITE_SIZE, ERASE_SIZE>
{
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        if offset as usize + bytes.len() > self.config.capacity {
            return Err(Error::OutOfRange);
        }
        yield_now().await;
        self.ensure_linear_mode();
        self.ll
            .0
            .write_spi_enable(SpiEnable::builder().with_enable(true).build());
        unsafe {
            core::ptr::copy_nonoverlapping(
                (QSPI_START_ADDRESS + offset as usize) as *const u8,
                bytes.as_mut_ptr(),
                bytes.len(),
            );
        }
        self.ll
            .0
            .write_spi_enable(SpiEnable::builder().with_enable(false).build());
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.capacity()
    }
}

impl<'d, T: Instance, const WRITE_SIZE: usize, const ERASE_SIZE: usize>
    embedded_storage_async::nor_flash::NorFlash for Qspi<'d, T, Async, WRITE_SIZE, ERASE_SIZE>
{
    const WRITE_SIZE: usize = WRITE_SIZE;
    const ERASE_SIZE: usize = ERASE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.erase_async(from, to).await
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_async(offset, bytes).await
    }
}

fn configure_qspi_pin<P: gpio::Pin>(
    pin: Peri<'_, P>,
    mux: gpio::MuxConfig,
    pullup: Option<bool>,
    voltage: IoType,
) {
    let pin = pin.into();
    let mut ll = gpio::LowLevelGpio::new(gpio::PinOffset::Mio(pin.offset() as usize));
    ll.configure_as_io_periph_pin(mux, pullup, Some(voltage));
}

fn enable_qspi_clock() {
    unsafe {
        slcr::with_unlocked(|slcr| {
            slcr.clk_ctrl().modify_aper_clk_ctrl(|mut val| {
                val.set_lqspi_1x_clk_act(true);
                val
            });
        });
    }
}

#[inline]
pub fn reset() {
    unsafe {
        slcr::with_unlocked(|regs| {
            regs.reset_ctrl().write_lqspi(
                ResetControlQspiSmc::builder()
                    .with_ref_reset(true)
                    .with_cpu_1x_reset(true)
                    .build(),
            );
            for _ in 0..10 {
                aarch32_cpu::asm::nop();
            }
            regs.reset_ctrl().write_lqspi(ResetControlQspiSmc::DEFAULT);
        });
    }
}

fn spi_mode_const_to_cpol_cpha(mode: SpiMode) -> (SpiClockPolarity, SpiClockPhase) {
    match mode {
        MODE_0 => (
            SpiClockPolarity::QuiescentLow,
            SpiClockPhase::ActiveOutsideOfWord,
        ),
        MODE_1 => (
            SpiClockPolarity::QuiescentLow,
            SpiClockPhase::InactiveOutsideOfWord,
        ),
        MODE_2 => (
            SpiClockPolarity::QuiescentHigh,
            SpiClockPhase::ActiveOutsideOfWord,
        ),
        MODE_3 => (
            SpiClockPolarity::QuiescentHigh,
            SpiClockPhase::InactiveOutsideOfWord,
        ),
    }
}

struct QspiIoTransferGuard<'a>(&'a mut QspiLowLevel);

impl<'a> QspiIoTransferGuard<'a> {
    fn new(qspi: &'a mut QspiLowLevel) -> Self {
        while qspi.regs().read_interrupt_status().rx_above_threshold() {
            qspi.regs().read_rx_data();
        }
        qspi.regs().modify_config(|mut val| {
            val.set_peripheral_chip_select(false);
            val
        });
        qspi.regs()
            .write_spi_enable(SpiEnable::builder().with_enable(true).build());
        Self(qspi)
    }

    fn start(&mut self) {
        self.0.regs().modify_config(|mut val| {
            val.set_manual_start_command(true);
            val
        });
    }

    fn write_word_txd_00(&mut self, word: u32) {
        self.0.regs().write_tx_data_00(word);
    }

    fn write_word_txd_01(&mut self, word: u32) {
        self.0.regs().write_tx_data_01(word);
    }

    fn write_word_txd_10(&mut self, word: u32) {
        self.0.regs().write_tx_data_10(word);
    }

    fn read_rx_data(&mut self) -> u32 {
        self.0.regs().read_rx_data()
    }

    fn read_status(&mut self) -> InterruptStatus {
        self.0.regs().read_interrupt_status()
    }
}

impl Deref for QspiIoTransferGuard<'_> {
    type Target = QspiLowLevel;

    fn deref(&self) -> &Self::Target {
        self.0
    }
}

impl DerefMut for QspiIoTransferGuard<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0
    }
}

impl Drop for QspiIoTransferGuard<'_> {
    fn drop(&mut self) {
        self.0.regs().modify_config(|mut val| {
            val.set_peripheral_chip_select(true);
            val
        });
        self.0
            .regs()
            .write_spi_enable(SpiEnable::builder().with_enable(false).build());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spansion_config_uses_32bit_addressing() {
        let cfg = Config::spansion_s25fl256s(
            ClockConfig::new(SrcSelIo::IoPll, u6::new(1), BaudRateConfig::WithLoopback),
            IoType::LvCmos33,
            lqspi_configs::RD_ONE,
        );
        assert_eq!(cfg.address_width, AddressWidth::Bits32);
    }

    #[test]
    fn common_linear_config_uses_vendor_specific_quad_io_dummy_cycles() {
        let config = linear_mode_config_for_common_devices(QspiDeviceCombination {
            vendor: QspiVendor::Micron,
            operating_mode: InstructionCode::FastReadQuadIo,
            two_devices: false,
        });
        assert_eq!(
            config.raw_value(),
            lqspi_configs::micron::QUAD_IO_FAST_RD_ONE.raw_value()
        );
    }

    #[test]
    fn address_width_reports_expected_command_words() {
        assert_eq!(AddressWidth::Bits24.command_words(), 1);
        assert_eq!(AddressWidth::Bits32.command_words(), 2);
    }

    #[test]
    fn calculate_with_ref_clk_rejects_zero_target_frequency() {
        assert!(matches!(
            ClockConfig::calculate_with_ref_clk(
                Hertz::from_raw(100_000_000),
                SrcSelIo::IoPll,
                Hertz::from_raw(0),
                BaudRateConfig::WithLoopback,
            ),
            Err(ClockCalculationError::ZeroTargetClock)
        ));
    }

    #[test]
    fn non_word_write_size_is_rejected() {
        assert_eq!(validate_write_size::<2>(), Err(Error::InvalidWriteSize));
        assert_eq!(validate_write_size::<4>(), Ok(()));
    }
}
