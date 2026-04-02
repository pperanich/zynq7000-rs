use core::{future::poll_fn, marker::PhantomData, task::Poll};

use arbitrary_int::{traits::Integer as _, u3, u4, u6, u12};
use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;
use zynq7000::{
    sdio::{
        BlockSelect, BusWidth, CommandRegister, InterruptMask, InterruptStatus, MmioRegisters,
        PresentState, ResponseType, SdClockDivisor,
    },
    slcr::{clocks::SrcSelIo, mio::IoType, reset::DualRefAndClockResetSdio},
};

use crate::{Hertz, gpio, interrupt, pac, slcr};

pub mod sd;

pub(crate) const MUX_CONF: gpio::MuxConfig = gpio::MuxConfig::new_with_l3(u3::new(0b100));
pub const BLOCK_LEN: usize = 512;
const MAX_DATA_TRANSFER_BYTES: usize = Adma2DescriptorTable::MAX_TRANSFER_BYTES;
pub(crate) const MAX_DMA_BLOCKS_PER_BATCH: usize = MAX_DATA_TRANSFER_BYTES / BLOCK_LEN;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SdioId {
    Sdio0,
    Sdio1,
}

impl SdioId {
    const fn index(self) -> usize {
        match self {
            Self::Sdio0 => 0,
            Self::Sdio1 => 1,
        }
    }
}

pub(crate) struct State {
    waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

pub(crate) static SDMMC_STATES: [State; 2] = [const { State::new() }, const { State::new() }];

#[doc(hidden)]
pub trait SealedInstance {
    fn id() -> SdioId;
    fn regs() -> pac::sdio::MmioRegisters<'static>;
    fn state() -> &'static crate::sdmmc::State;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    type Interrupt: interrupt::typelevel::Interrupt;
}

pub(crate) mod sealed {
    pub trait RouteGroup {}

    pub trait ClockPin<T> {
        type RouteGroup: RouteGroup;
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait CommandPin<T> {
        type RouteGroup: RouteGroup;
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait Data0Pin<T> {
        type RouteGroup: RouteGroup;
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait Data1Pin<T> {
        type RouteGroup: RouteGroup;
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait Data2Pin<T> {
        type RouteGroup: RouteGroup;
        fn mux_config() -> crate::gpio::MuxConfig;
    }
    pub trait Data3Pin<T> {
        type RouteGroup: RouteGroup;
        fn mux_config() -> crate::gpio::MuxConfig;
    }

    pub trait Bus1Bit<T, CLK, CMD, D0> {}
    pub trait Bus4Bit<T, CLK, CMD, D0, D1, D2, D3> {}
}

pub trait ClockPin<T: Instance>: gpio::Pin + sealed::ClockPin<T> {}
pub trait CommandPin<T: Instance>: gpio::Pin + sealed::CommandPin<T> {}
pub trait Data0Pin<T: Instance>: gpio::Pin + sealed::Data0Pin<T> {}
pub trait Data1Pin<T: Instance>: gpio::Pin + sealed::Data1Pin<T> {}
pub trait Data2Pin<T: Instance>: gpio::Pin + sealed::Data2Pin<T> {}
pub trait Data3Pin<T: Instance>: gpio::Pin + sealed::Data3Pin<T> {}
pub trait Bus1Bit<T: Instance, CLK: ClockPin<T>, CMD: CommandPin<T>, D0: Data0Pin<T>>:
    sealed::Bus1Bit<T, CLK, CMD, D0>
{
}
pub trait Bus4Bit<
    T: Instance,
    CLK: ClockPin<T>,
    CMD: CommandPin<T>,
    D0: Data0Pin<T>,
    D1: Data1Pin<T>,
    D2: Data2Pin<T>,
    D3: Data3Pin<T>,
>: sealed::Bus4Bit<T, CLK, CMD, D0, D1, D2, D3>
{
}

#[derive(Debug, Clone, Copy)]
pub struct SdioDivisors {
    pub divisor_init_phase: SdClockDivisor,
    pub divisor_normal: SdClockDivisor,
}

impl SdioDivisors {
    pub fn calculate(ref_clk: Hertz, target_speed: Hertz) -> Self {
        const INIT_CLOCK_HZ: u32 = 400_000;
        let divisor_select_from_value = |value: u32| match value {
            0..=1 => SdClockDivisor::Div1,
            2 => SdClockDivisor::Div2,
            3..=4 => SdClockDivisor::Div4,
            5..=8 => SdClockDivisor::Div8,
            9..=16 => SdClockDivisor::Div16,
            17..=32 => SdClockDivisor::Div32,
            33..=64 => SdClockDivisor::Div64,
            65..=128 => SdClockDivisor::Div128,
            _ => SdClockDivisor::Div256,
        };
        Self {
            divisor_init_phase: divisor_select_from_value(ref_clk.raw().div_ceil(INIT_CLOCK_HZ)),
            divisor_normal: divisor_select_from_value(ref_clk.raw().div_ceil(target_speed.raw())),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub src_sel: SrcSelIo,
    pub ref_clock_divisor: u6,
    pub sdio_clock_divisors: SdioDivisors,
    pub high_speed: bool,
    pub io_type: IoType,
}

impl Config {
    pub const fn new(
        src_sel: SrcSelIo,
        ref_clock_divisor: u6,
        sdio_clock_divisors: SdioDivisors,
        high_speed: bool,
        io_type: IoType,
    ) -> Self {
        Self {
            src_sel,
            ref_clock_divisor,
            sdio_clock_divisors,
            high_speed,
            io_type,
        }
    }

    pub fn calculate_for_io_clock(
        io_clocks: &crate::clocks::IoClocks,
        src_sel: SrcSelIo,
        target_ref_clock: Hertz,
        target_sd_speed: Hertz,
        io_type: IoType,
    ) -> Option<Self> {
        let ref_clk = io_clocks.selected_ref_clk(src_sel);
        let io_ref_clock_divisor = ref_clk.raw().div_ceil(target_ref_clock.raw());
        if io_ref_clock_divisor > u6::MAX.as_u32() {
            return None;
        }
        let target_speed = ref_clk / io_ref_clock_divisor;

        Some(Self {
            src_sel,
            ref_clock_divisor: u6::new(io_ref_clock_divisor as u8),
            sdio_clock_divisors: SdioDivisors::calculate(target_speed, target_sd_speed),
            high_speed: target_sd_speed.raw() > 25_000_000,
            io_type,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusMode {
    OneBit,
    FourBit,
}

#[derive(Debug, thiserror::Error, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    #[error("response error")]
    ResponseError(ResponseErrorBits),
    #[error("data error")]
    DataError(DataErrorBits),
    #[error("adma error")]
    AdmaError(AdmaErrorBits),
    #[error("unexpected response")]
    UnexpectedResponse,
    #[error("no card present")]
    NoCard,
    #[error("invalid block size")]
    InvalidBlockSize,
    #[error("too many blocks for single transfer")]
    TooManyBlocks,
    #[error("unsupported card version")]
    UnsupportedCardVersion,
    #[error("unsupported voltage")]
    UnsupportedVoltage,
    #[error("invalid peripheral instance")]
    InvalidPeripheral,
    #[error("cache maintenance alignment error")]
    CacheAlignment,
    #[error("cache maintenance is unavailable")]
    CacheUnavailable,
    #[error("SD transfer exceeds ADMA descriptor capacity")]
    TransferTooLarge,
    #[error("timed out waiting for card initialization")]
    CardInitTimeout,
    #[error("timed out waiting for card programming completion")]
    ProgrammingTimeout,
}

impl From<crate::cache::AlignmentError> for Error {
    fn from(value: crate::cache::AlignmentError) -> Self {
        match value {
            crate::cache::AlignmentError::Unaligned => Error::CacheAlignment,
            crate::cache::AlignmentError::Unavailable => Error::CacheUnavailable,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct ResponseErrorBits {
    index: bool,
    crc: bool,
    end_bit: bool,
    timeout: bool,
}

impl ResponseErrorBits {
    pub const fn has_error(&self) -> bool {
        self.index || self.crc || self.end_bit || self.timeout
    }

    pub const fn timeout(&self) -> bool {
        self.timeout
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct DataErrorBits {
    crc: bool,
    timeout: bool,
    end_bit: bool,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct AdmaErrorBits {
    length_mismatch: bool,
    state: zynq7000::sdio::AdmaErrorState,
}

impl From<zynq7000::sdio::AdmaErrorStatus> for AdmaErrorBits {
    fn from(value: zynq7000::sdio::AdmaErrorStatus) -> Self {
        Self {
            length_mismatch: value.length_mismatch_error(),
            state: value.error_state(),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct StatusWrapper(pub InterruptStatus);

impl StatusWrapper {
    pub fn response_errors(&self) -> ResponseErrorBits {
        ResponseErrorBits {
            index: self.0.command_index_error(),
            crc: self.0.command_crc_error(),
            end_bit: self.0.command_end_bit_error(),
            timeout: self.0.command_timeout_error(),
        }
    }

    pub fn has_response_errors(&self) -> bool {
        self.response_errors().has_error()
    }

    pub fn data_errors(&self) -> DataErrorBits {
        DataErrorBits {
            crc: self.0.data_crc_error(),
            timeout: self.0.data_timeout_error(),
            end_bit: self.0.data_end_bit_error(),
        }
    }

    pub fn has_data_error(&self) -> bool {
        self.0.data_timeout_error() || self.0.data_crc_error() || self.0.data_end_bit_error()
    }
}

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::regs().read_interrupt_status();
        T::regs().write_interrupt_signal_enable(InterruptMask::ZERO);
        T::state().waker.wake();
    }
}

pub struct Sdmmc<'d, T: Instance> {
    regs: MmioRegisters<'static>,
    state: &'static State,
    bus_mode: BusMode,
    config: Config,
    adma_table: CacheAlignedAdma2DescriptorTable,
    _phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> Sdmmc<'d, T> {
    pub fn new_1bit<CLK: ClockPin<T>, CMD: CommandPin<T>, D0: Data0Pin<T>>(
        _peri: Peri<'d, T>,
        irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        clk: Peri<'d, CLK>,
        cmd: Peri<'d, CMD>,
        d0: Peri<'d, D0>,
        config: Config,
    ) -> Self
    where
        (): Bus1Bit<T, CLK, CMD, D0>,
    {
        configure_clock_pin::<T, CLK>(clk, config.io_type);
        configure_command_pin::<T, CMD>(cmd, config.io_type);
        configure_data0_pin::<T, D0>(d0, config.io_type);
        interrupt::bind::<T::Interrupt, InterruptHandler<T>, _>(irq);
        Self::new_inner(config, BusMode::OneBit)
    }

    pub fn new_4bit<
        CLK: ClockPin<T>,
        CMD: CommandPin<T>,
        D0: Data0Pin<T>,
        D1: Data1Pin<T>,
        D2: Data2Pin<T>,
        D3: Data3Pin<T>,
    >(
        _peri: Peri<'d, T>,
        irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        clk: Peri<'d, CLK>,
        cmd: Peri<'d, CMD>,
        d0: Peri<'d, D0>,
        d1: Peri<'d, D1>,
        d2: Peri<'d, D2>,
        d3: Peri<'d, D3>,
        config: Config,
    ) -> Self
    where
        (): Bus4Bit<T, CLK, CMD, D0, D1, D2, D3>,
    {
        configure_clock_pin::<T, CLK>(clk, config.io_type);
        configure_command_pin::<T, CMD>(cmd, config.io_type);
        configure_data0_pin::<T, D0>(d0, config.io_type);
        configure_data1_pin::<T, D1>(d1, config.io_type);
        configure_data2_pin::<T, D2>(d2, config.io_type);
        configure_data3_pin::<T, D3>(d3, config.io_type);
        interrupt::bind::<T::Interrupt, InterruptHandler<T>, _>(irq);
        Self::new_inner(config, BusMode::FourBit)
    }

    fn new_inner(config: Config, bus_mode: BusMode) -> Self {
        let regs = T::regs();
        let mut this = Self {
            regs,
            state: T::state(),
            bus_mode,
            config,
            adma_table: CacheAlignedAdma2DescriptorTable::new(Adma2DescriptorTable::new()),
            _phantom: PhantomData,
        };
        this.initialize();
        this
    }

    pub(crate) fn bus_mode(&self) -> BusMode {
        self.bus_mode
    }

    pub(crate) fn config(&self) -> Config {
        self.config
    }

    pub(crate) fn read_present_state(&self) -> PresentState {
        self.regs.read_present_state()
    }

    pub(crate) fn capabilities(&self) -> zynq7000::sdio::Capabilities {
        self.regs.read_capabilities()
    }

    pub(crate) fn read_u32_response(&self) -> u32 {
        self.regs.read_responses(0).unwrap()
    }

    pub(crate) fn read_u128_response(&self) -> [u8; 16] {
        let mut words = [0u32; 4];
        for (index, word) in words.iter_mut().rev().enumerate() {
            *word = self.regs.read_responses(index).unwrap().to_be();
        }
        let mut bytes = [0u8; 16];
        for (index, word) in words.iter().enumerate() {
            bytes[index * 4..(index + 1) * 4].copy_from_slice(&word.to_ne_bytes());
        }
        bytes.copy_within(1.., 0);
        bytes[15] = 0;
        bytes
    }

    pub(crate) fn clear_all_status_bits(&mut self) {
        self.regs.write_interrupt_status(InterruptStatus::ack_all());
    }

    pub(crate) fn clear_status_bits(&mut self, status: InterruptStatus) {
        self.regs
            .write_interrupt_status(InterruptStatus::ack_from(status));
    }

    fn write_command(&mut self, command: CommandRegister, arg: u32) {
        self.clear_all_status_bits();
        self.regs.write_argument(arg);
        self.regs.write_command(command);
    }

    pub(crate) async fn send_command(
        &mut self,
        command: CommandRegister,
        arg: u32,
    ) -> Result<StatusWrapper, Error> {
        self.write_command(command, arg);
        let status = self.wait_for_command_complete().await;
        if status.has_response_errors() {
            return Err(Error::ResponseError(status.response_errors()));
        }
        Ok(status)
    }

    pub(crate) async fn transfer_read(
        &mut self,
        command: CommandRegister,
        arg: u32,
        buffer: &mut [u8],
        blocks: usize,
    ) -> Result<(), Error> {
        self.prepare_read_transfer(buffer, blocks, BLOCK_LEN)?;
        self.send_command(command, arg).await?;
        self.wait_for_transfer_complete().await?;
        self.complete_read_transfer(buffer)?;
        Ok(())
    }

    pub(crate) async fn transfer_read_with_block_len(
        &mut self,
        command: CommandRegister,
        arg: u32,
        buffer: &mut [u8],
        blocks: usize,
        block_len: usize,
    ) -> Result<(), Error> {
        self.prepare_read_transfer(buffer, blocks, block_len)?;
        self.send_command(command, arg).await?;
        self.wait_for_transfer_complete().await?;
        self.complete_read_transfer(buffer)?;
        Ok(())
    }

    pub(crate) async fn transfer_write(
        &mut self,
        command: CommandRegister,
        arg: u32,
        buffer: &[u8],
        blocks: usize,
    ) -> Result<(), Error> {
        self.prepare_write_transfer(buffer, blocks)?;
        self.send_command(command, arg).await?;
        self.wait_for_transfer_complete().await
    }

    pub(crate) fn set_card_bus_width(&mut self, bus_mode: BusMode) {
        self.regs
            .modify_host_power_blockgap_wakeup_control(|mut val| {
                val.set_bus_width(match bus_mode {
                    BusMode::OneBit => BusWidth::_1bit,
                    BusMode::FourBit => BusWidth::_4bits,
                });
                val.set_high_speed_enable(self.config.high_speed);
                val
            });
    }

    pub(crate) fn switch_to_normal_transfer_clock(&mut self) {
        self.disable_sd_clock();
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_sdclk_frequency_select(self.config.sdio_clock_divisors.divisor_normal);
            val
        });
        self.enable_sd_clock();
    }

    fn wait_signal_mask_command() -> InterruptMask {
        InterruptMask::ZERO
            .with_command_complete(true)
            .with_command_timeout_error(true)
            .with_command_crc_error(true)
            .with_command_end_bit_error(true)
            .with_command_index_error(true)
    }

    fn wait_signal_mask_transfer() -> InterruptMask {
        InterruptMask::ZERO
            .with_transfer_complete(true)
            .with_dma_interrupt(true)
            .with_data_timeout_error(true)
            .with_data_crc_error(true)
            .with_data_end_bit_error(true)
            .with_adma_error(true)
            .with_auto_cmd12_error(true)
            .with_target_response_error(true)
            .with_current_limit_error(true)
            .with_ceata_error_status(true)
    }

    async fn wait_for_command_complete(&mut self) -> StatusWrapper {
        poll_fn(|cx| {
            self.state.waker.register(cx.waker());
            let status = self.regs.read_interrupt_status();
            if status.command_complete() || status.error_interrupt() {
                self.regs.write_interrupt_signal_enable(InterruptMask::ZERO);
                self.clear_status_bits(status);
                return Poll::Ready(StatusWrapper(status));
            }
            self.regs
                .write_interrupt_signal_enable(Self::wait_signal_mask_command());
            Poll::Pending
        })
        .await
    }

    async fn wait_for_transfer_complete(&mut self) -> Result<(), Error> {
        poll_fn(|cx| {
            self.state.waker.register(cx.waker());
            let status = self.regs.read_interrupt_status();
            if status.dma_interrupt() {
                self.clear_status_bits(InterruptStatus::ZERO.with_dma_interrupt(true));
            }
            if status.transfer_complete()
                || status.adma_error()
                || status.error_interrupt()
                || status.auto_cmd12_error()
                || status.target_response_error()
                || status.current_limit_error()
                || status.ceata_error_status()
            {
                self.regs.write_interrupt_signal_enable(InterruptMask::ZERO);
                self.clear_status_bits(status);
                if status.adma_error() {
                    let adma_status = self.regs.read_adma_error_status();
                    self.regs
                        .write_adma_error_status(zynq7000::sdio::AdmaErrorStatus::clear_error());
                    return Poll::Ready(Err(Error::AdmaError(adma_status.into())));
                }
                if status.data_timeout_error()
                    || status.data_crc_error()
                    || status.data_end_bit_error()
                {
                    return Poll::Ready(Err(Error::DataError(StatusWrapper(status).data_errors())));
                }
                if status.auto_cmd12_error()
                    || status.target_response_error()
                    || status.current_limit_error()
                    || status.ceata_error_status()
                {
                    return Poll::Ready(Err(Error::UnexpectedResponse));
                }
                return Poll::Ready(Ok(()));
            }
            self.regs
                .write_interrupt_signal_enable(Self::wait_signal_mask_transfer());
            Poll::Pending
        })
        .await
    }

    fn initialize(&mut self) {
        enable_amba_peripheral_clock(T::id());
        self.reset(5);

        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_software_reset_for_all(true);
            val
        });
        while self
            .regs
            .read_clock_timeout_sw_reset_control()
            .software_reset_for_all()
        {}

        self.configure_clock();
        self.enable_internal_clock();
        while !self
            .regs
            .read_clock_timeout_sw_reset_control()
            .internal_clock_stable()
        {}

        self.regs
            .modify_host_power_blockgap_wakeup_control(|mut val| {
                val.set_sd_bus_power(true);
                val.set_sd_bus_voltage_select(zynq7000::sdio::SdBusVoltageSelect::_3_3V);
                val.set_dma_select(zynq7000::sdio::DmaSelect::Adma2_32bits);
                val.set_bus_width(BusWidth::_1bit);
                val.set_high_speed_enable(false);
                val
            });

        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_data_timeout_counter_value(u4::new(0xE));
            val
        });
        self.regs.modify_block_params(|mut val| {
            val.set_transfer_block_size(u12::new(BLOCK_LEN as u16));
            val
        });
        self.regs
            .write_interrupt_status_enable(InterruptMask::all_enabled_without_card_interrupt());
        self.regs.write_interrupt_signal_enable(InterruptMask::ZERO);
        self.clear_all_status_bits();
        self.enable_sd_clock();
    }

    fn configure_clock(&mut self) {
        unsafe {
            slcr::with_unlocked(|slcr| {
                slcr.clk_ctrl().modify_sdio_clk_ctrl(|mut val| {
                    val.set_srcsel(self.config.src_sel);
                    val.set_divisor(self.config.ref_clock_divisor);
                    match T::id() {
                        SdioId::Sdio0 => val.set_clk_0_act(true),
                        SdioId::Sdio1 => val.set_clk_1_act(true),
                    }
                    val
                });
            });
        }
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_sdclk_frequency_select(self.config.sdio_clock_divisors.divisor_init_phase);
            val
        });
    }

    fn prepare_read_transfer(
        &mut self,
        buffer: &mut [u8],
        blocks: usize,
        block_len: usize,
    ) -> Result<(), Error> {
        if !buffer.len().is_multiple_of(block_len) || buffer.len() != blocks * block_len {
            return Err(Error::InvalidBlockSize);
        }
        let (addr, len) = cache_maintenance_range(buffer.as_mut_ptr() as usize, buffer.len());
        if len != 0 {
            crate::cache::clean_and_invalidate_data_cache_range(addr, len)?;
        }
        self.configure_data_blocks(blocks, block_len)?;
        self.prepare_adma_transfer(buffer.as_mut_ptr() as u32, buffer.len())?;
        Ok(())
    }

    fn prepare_write_transfer(&mut self, buffer: &[u8], blocks: usize) -> Result<(), Error> {
        if !buffer.len().is_multiple_of(BLOCK_LEN) || buffer.len() != blocks * BLOCK_LEN {
            return Err(Error::InvalidBlockSize);
        }
        let (addr, len) = cache_maintenance_range(buffer.as_ptr() as usize, buffer.len());
        if len != 0 {
            crate::cache::clean_data_cache_range(addr, len)?;
        }
        self.configure_data_blocks(blocks, BLOCK_LEN)?;
        self.prepare_adma_transfer(buffer.as_ptr() as u32, buffer.len())?;
        Ok(())
    }

    fn configure_data_blocks(&mut self, blocks: usize, block_len: usize) -> Result<(), Error> {
        let block_count = u16::try_from(blocks).map_err(|_| Error::TooManyBlocks)?;
        self.regs.modify_block_params(|mut val| {
            val.set_transfer_block_size(u12::new(block_len as u16));
            val.set_block_counts_for_current_transfer(block_count);
            val
        });
        Ok(())
    }

    fn complete_read_transfer(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let (addr, len) = cache_maintenance_range(buffer.as_mut_ptr() as usize, buffer.len());
        if len != 0 {
            crate::cache::invalidate_data_cache_range(addr, len)?;
        }
        Ok(())
    }

    fn prepare_adma_transfer(&mut self, buffer_addr: u32, len: usize) -> Result<(), Error> {
        if len > Adma2DescriptorTable::MAX_TRANSFER_BYTES {
            return Err(Error::TransferTooLarge);
        }
        self.adma_table.0.configure(buffer_addr, len);
        let (addr, span) = cache_maintenance_range(
            core::ptr::from_ref(&self.adma_table) as usize,
            core::mem::size_of_val(&self.adma_table),
        );
        if span != 0 {
            crate::cache::clean_data_cache_range(addr, span)?;
        }
        self.regs
            .write_adma_system_address(self.adma_table.0.as_ptr() as u32);
        Ok(())
    }

    fn enable_internal_clock(&mut self) {
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_internal_clock_enable(true);
            val
        });
    }

    fn enable_sd_clock(&mut self) {
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_sd_clock_enable(true);
            val
        });
    }

    fn disable_sd_clock(&mut self) {
        self.regs.modify_clock_timeout_sw_reset_control(|mut val| {
            val.set_sd_clock_enable(false);
            val
        });
    }

    fn reset(&mut self, cycles: u32) {
        reset(T::id(), cycles);
    }
}

fn enable_amba_peripheral_clock(id: SdioId) {
    unsafe {
        slcr::with_unlocked(|slcr| {
            slcr.clk_ctrl().modify_aper_clk_ctrl(|mut val| {
                match id {
                    SdioId::Sdio0 => val.set_sdio_0_1x_clk_act(true),
                    SdioId::Sdio1 => val.set_sdio_1_1x_clk_act(true),
                }
                val
            });
        });
    }
}

pub fn reset(id: SdioId, cycles: u32) {
    let assert_reset = match id {
        SdioId::Sdio0 => DualRefAndClockResetSdio::builder()
            .with_periph0_ref_rst(true)
            .with_periph1_ref_rst(false)
            .with_periph0_cpu1x_rst(true)
            .with_periph1_cpu1x_rst(false)
            .build(),
        SdioId::Sdio1 => DualRefAndClockResetSdio::builder()
            .with_periph0_ref_rst(false)
            .with_periph1_ref_rst(true)
            .with_periph0_cpu1x_rst(false)
            .with_periph1_cpu1x_rst(true)
            .build(),
    };
    unsafe {
        slcr::with_unlocked(|regs| {
            regs.reset_ctrl().write_sdio(assert_reset);
            for _ in 0..cycles {
                aarch32_cpu::asm::nop();
            }
            regs.reset_ctrl().write_sdio(DualRefAndClockResetSdio::ZERO);
        });
    }
}

fn configure_clock_pin<T: Instance, P: ClockPin<T>>(pin: Peri<'_, P>, io_type: IoType) {
    configure_peripheral_pin::<P>(pin, <P as sealed::ClockPin<T>>::mux_config(), io_type);
}

fn configure_command_pin<T: Instance, P: CommandPin<T>>(pin: Peri<'_, P>, io_type: IoType) {
    configure_peripheral_pin::<P>(pin, <P as sealed::CommandPin<T>>::mux_config(), io_type);
}

fn configure_data0_pin<T: Instance, P: Data0Pin<T>>(pin: Peri<'_, P>, io_type: IoType) {
    configure_peripheral_pin::<P>(pin, <P as sealed::Data0Pin<T>>::mux_config(), io_type);
}

fn configure_data1_pin<T: Instance, P: Data1Pin<T>>(pin: Peri<'_, P>, io_type: IoType) {
    configure_peripheral_pin::<P>(pin, <P as sealed::Data1Pin<T>>::mux_config(), io_type);
}

fn configure_data2_pin<T: Instance, P: Data2Pin<T>>(pin: Peri<'_, P>, io_type: IoType) {
    configure_peripheral_pin::<P>(pin, <P as sealed::Data2Pin<T>>::mux_config(), io_type);
}

fn configure_data3_pin<T: Instance, P: Data3Pin<T>>(pin: Peri<'_, P>, io_type: IoType) {
    configure_peripheral_pin::<P>(pin, <P as sealed::Data3Pin<T>>::mux_config(), io_type);
}

fn configure_peripheral_pin<P: gpio::Pin>(
    pin: Peri<'_, P>,
    mux_conf: gpio::MuxConfig,
    io_type: IoType,
) {
    let pin = pin.into();
    let mut ll = gpio::LowLevelGpio::new(gpio::PinOffset::Mio(pin.offset() as usize));
    ll.configure_as_io_periph_pin(mux_conf, Some(false), Some(io_type));
}

pub(crate) const fn cache_maintenance_range(addr: usize, len: usize) -> (u32, usize) {
    if len == 0 {
        return (addr as u32, 0);
    }
    let start = addr & !(crate::cache::CACHE_LINE_SIZE - 1);
    let end =
        (addr + len + crate::cache::CACHE_LINE_SIZE - 1) & !(crate::cache::CACHE_LINE_SIZE - 1);
    (start as u32, end - start)
}

const DESC_MAX_LENGTH: usize = 65_536;
const ATTR_TRANSFER: u16 = 1 << 5;
const ATTR_END: u16 = 1 << 1;
const ATTR_VALID: u16 = 1 << 0;
const ADMA2_DESCRIPTOR_COUNT: usize = 32;

#[repr(C, align(4))]
#[derive(Debug, Clone, Copy, Default)]
struct Adma2Descriptor32 {
    attribute: u16,
    length: u16,
    address: u32,
}

impl Adma2Descriptor32 {
    const fn new() -> Self {
        Self {
            attribute: 0,
            length: 0,
            address: 0,
        }
    }

    fn configure(&mut self, address: u32, len: usize, end: bool) {
        self.address = address;
        self.length = if len == DESC_MAX_LENGTH {
            0
        } else {
            len as u16
        };
        self.attribute = ATTR_TRANSFER | ATTR_VALID | if end { ATTR_END } else { 0 };
    }
}

#[derive(Debug, Clone, Copy)]
struct Adma2DescriptorTable {
    entries: [Adma2Descriptor32; ADMA2_DESCRIPTOR_COUNT],
}

impl Adma2DescriptorTable {
    const MAX_TRANSFER_BYTES: usize = ADMA2_DESCRIPTOR_COUNT * DESC_MAX_LENGTH;

    const fn new() -> Self {
        Self {
            entries: [const { Adma2Descriptor32::new() }; ADMA2_DESCRIPTOR_COUNT],
        }
    }

    fn configure(&mut self, buffer_addr: u32, len: usize) {
        self.entries.fill(Adma2Descriptor32::new());
        let descriptor_count = len.div_ceil(DESC_MAX_LENGTH);
        for desc_idx in 0..descriptor_count {
            let offset = desc_idx * DESC_MAX_LENGTH;
            let desc_len = core::cmp::min(DESC_MAX_LENGTH, len - offset);
            self.entries[desc_idx].configure(
                buffer_addr + offset as u32,
                desc_len,
                desc_idx + 1 == descriptor_count,
            );
        }
    }

    fn as_ptr(&self) -> *const Adma2Descriptor32 {
        self.entries.as_ptr()
    }
}

#[repr(C, align(32))]
struct CacheAlignedAdma2DescriptorTable(Adma2DescriptorTable);

impl CacheAlignedAdma2DescriptorTable {
    const fn new(inner: Adma2DescriptorTable) -> Self {
        Self(inner)
    }
}

pub(crate) const fn build_command_without_data(
    id: u8,
    response_type: ResponseType,
    index_check: bool,
    crc_check: bool,
) -> CommandRegister {
    CommandRegister::builder()
        .with_command_index(arbitrary_int::u6::new(id))
        .with_command_type(zynq7000::sdio::CommandType::Normal)
        .with_data_is_present(false)
        .with_command_index_check_enable(index_check)
        .with_command_crc_check_enable(crc_check)
        .with_response_type_select(response_type)
        .with_block_select(BlockSelect::SingleBlock)
        .with_data_transfer_direction(zynq7000::sdio::TransferDirection::Write)
        .with_auto_cmd12_enable(false)
        .with_block_count_enable(false)
        .with_dma_enable(false)
        .build()
}

pub(crate) const fn build_data_command(
    id: u8,
    block_select: BlockSelect,
    direction: zynq7000::sdio::TransferDirection,
    auto_cmd12: bool,
    block_count: bool,
) -> CommandRegister {
    CommandRegister::builder()
        .with_command_index(arbitrary_int::u6::new(id))
        .with_command_type(zynq7000::sdio::CommandType::Normal)
        .with_data_is_present(true)
        .with_command_index_check_enable(true)
        .with_command_crc_check_enable(true)
        .with_response_type_select(ResponseType::_48bits)
        .with_block_select(block_select)
        .with_data_transfer_direction(direction)
        .with_auto_cmd12_enable(auto_cmd12)
        .with_block_count_enable(block_count)
        .with_dma_enable(true)
        .build()
}

pub(crate) const CMD0_GO_IDLE_MODE: CommandRegister =
    build_command_without_data(0, ResponseType::None, false, false);
pub(crate) const CMD2_ALL_SEND_CID: CommandRegister =
    build_command_without_data(2, ResponseType::_136bits, false, true);
pub(crate) const CMD3_SEND_RELATIVE_ADDR: CommandRegister =
    build_command_without_data(3, ResponseType::_48bitsWithCheck, false, false);
pub(crate) const CMD7_SELECT_SD_CARD: CommandRegister =
    build_command_without_data(7, ResponseType::_48bits, true, true);
pub(crate) const CMD8_SEND_IF_COND: CommandRegister =
    build_command_without_data(8, ResponseType::_48bits, true, true);
pub(crate) const CMD9_SEND_CSD: CommandRegister =
    build_command_without_data(9, ResponseType::_136bits, false, true);
pub(crate) const CMD13_SEND_STATUS: CommandRegister =
    build_command_without_data(13, ResponseType::_48bits, true, true);
pub(crate) const CMD16_SET_BLOCKLEN: CommandRegister =
    build_command_without_data(16, ResponseType::_48bits, true, true);
pub(crate) const CMD55_APP_CMD: CommandRegister =
    build_command_without_data(55, ResponseType::_48bits, true, true);
pub(crate) const ACMD6_SET_BUS_WIDTH: CommandRegister =
    build_command_without_data(6, ResponseType::_48bits, true, true);
pub(crate) const ACMD41_SEND_OP_COND: CommandRegister =
    build_command_without_data(41, ResponseType::_48bits, false, false);
pub(crate) const ACMD51_SEND_SCR: CommandRegister = build_data_command(
    51,
    BlockSelect::SingleBlock,
    zynq7000::sdio::TransferDirection::Read,
    false,
    false,
);
pub(crate) const CMD17_READ_SINGLE_BLOCK: CommandRegister = build_data_command(
    17,
    BlockSelect::SingleBlock,
    zynq7000::sdio::TransferDirection::Read,
    false,
    false,
);
pub(crate) const CMD18_READ_MULTIPLE_BLOCKS: CommandRegister = build_data_command(
    18,
    BlockSelect::MultiBlock,
    zynq7000::sdio::TransferDirection::Read,
    true,
    true,
);
pub(crate) const CMD24_WRITE_BLOCK: CommandRegister = build_data_command(
    24,
    BlockSelect::SingleBlock,
    zynq7000::sdio::TransferDirection::Write,
    false,
    false,
);
pub(crate) const CMD25_WRITE_MULTIPLE_BLOCKS: CommandRegister = build_data_command(
    25,
    BlockSelect::MultiBlock,
    zynq7000::sdio::TransferDirection::Write,
    true,
    true,
);
