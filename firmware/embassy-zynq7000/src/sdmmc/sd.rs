use core::ops::{Deref, DerefMut};

use embedded_sdmmc::sdcard::{
    CardType,
    argument::{self, OcrLower},
    csd::{Csd, InvalidCsdStructureFieldError},
    response::{self, R1, R6, R7},
};

use super::{
    ACMD6_SET_BUS_WIDTH, ACMD41_SEND_OP_COND, ACMD51_SEND_SCR, BLOCK_LEN, BusMode,
    CMD0_GO_IDLE_MODE, CMD2_ALL_SEND_CID, CMD3_SEND_RELATIVE_ADDR, CMD7_SELECT_SD_CARD,
    CMD8_SEND_IF_COND, CMD9_SEND_CSD, CMD13_SEND_STATUS, CMD16_SET_BLOCKLEN,
    CMD17_READ_SINGLE_BLOCK, CMD18_READ_MULTIPLE_BLOCKS, CMD24_WRITE_BLOCK,
    CMD25_WRITE_MULTIPLE_BLOCKS, CMD55_APP_CMD, Error, MAX_DMA_BLOCKS_PER_BATCH, Sdmmc,
};
use crate::{clocks, gtc::GlobalTimerCounter};

const CMD8_RETRIES: usize = 2;
const SCR_LEN: usize = 8;
const SCR_BUS_WIDTH_4BIT_MASK: u8 = 0b0100;
const CARD_INIT_TIMEOUT_MS: u64 = 1000;
const PROGRAMMING_TIMEOUT_MS: u64 = 1000;

pub const VOLTAGE_LEVEL_CAPABILITIES: OcrLower = OcrLower::builder()
    .with__3_5_to_3_6v(false)
    .with__3_4_to_3_5v(false)
    .with__3_3_to_3_4v(false)
    .with__3_2_to_3_3v(true)
    .with__3_1_to_3_2v(false)
    .with__3_0_to_3_1v(false)
    .with__2_9_to_3_0v(false)
    .with__2_8_to_2_9v(false)
    .with__2_7_to_2_8v(false)
    .with_reserved_low_voltage(false)
    .build();

#[derive(Debug, Clone)]
pub struct SdCardInfo {
    card_type: CardType,
    rca: u16,
    cid: embedded_sdmmc::sdcard::cid::Cid,
    csd: embedded_sdmmc::sdcard::csd::Csd,
}

impl SdCardInfo {
    pub fn card_type(&self) -> CardType {
        self.card_type
    }

    pub fn rca(&self) -> u16 {
        self.rca
    }

    pub fn cid(&self) -> &embedded_sdmmc::sdcard::cid::Cid {
        &self.cid
    }

    pub fn csd(&self) -> &embedded_sdmmc::sdcard::csd::Csd {
        &self.csd
    }
}

#[repr(align(32))]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DataBlock(pub [u8; BLOCK_LEN]);

impl DataBlock {
    pub const fn new() -> Self {
        Self([0; BLOCK_LEN])
    }
}

impl Deref for DataBlock {
    type Target = [u8; BLOCK_LEN];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for DataBlock {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

pub struct StorageDevice<'a, 'd, T: super::Instance> {
    info: SdCardInfo,
    pub sdmmc: &'a mut Sdmmc<'d, T>,
}

impl<'a, 'd, T: super::Instance> StorageDevice<'a, 'd, T> {
    pub async fn new_sd_card(sdmmc: &'a mut Sdmmc<'d, T>) -> Result<Self, Error> {
        let info = acquire_card(sdmmc).await?;
        Ok(Self { info, sdmmc })
    }

    pub async fn reacquire(&mut self) -> Result<(), Error> {
        self.info = acquire_card(self.sdmmc).await?;
        Ok(())
    }

    pub fn card_info(&self) -> &SdCardInfo {
        &self.info
    }

    pub async fn read_block(
        &mut self,
        block_idx: u32,
        data_block: &mut DataBlock,
    ) -> Result<(), Error> {
        let address = self.block_address(block_idx);
        self.sdmmc
            .transfer_read(CMD17_READ_SINGLE_BLOCK, address, &mut data_block.0, 1)
            .await
    }

    pub async fn read_blocks(
        &mut self,
        block_idx: u32,
        blocks: &mut [DataBlock],
    ) -> Result<(), Error> {
        let mut remaining = blocks;
        let mut current = block_idx;
        while !remaining.is_empty() {
            let batch_blocks = core::cmp::min(remaining.len(), MAX_DMA_BLOCKS_PER_BATCH);
            let (batch, rest) = remaining.split_at_mut(batch_blocks);
            let buffer = unsafe {
                core::slice::from_raw_parts_mut(
                    batch.as_mut_ptr().cast::<u8>(),
                    batch.len() * BLOCK_LEN,
                )
            };
            let address = self.block_address(current);
            let command = if batch.len() == 1 {
                CMD17_READ_SINGLE_BLOCK
            } else {
                CMD18_READ_MULTIPLE_BLOCKS
            };
            self.sdmmc
                .transfer_read(command, address, buffer, batch.len())
                .await?;
            current = current.saturating_add(batch.len() as u32);
            remaining = rest;
        }
        Ok(())
    }

    pub async fn write_block(
        &mut self,
        block_idx: u32,
        data_block: &DataBlock,
    ) -> Result<(), Error> {
        let address = self.block_address(block_idx);
        self.sdmmc
            .transfer_write(CMD24_WRITE_BLOCK, address, &data_block.0, 1)
            .await?;
        self.wait_for_programming_done().await
    }

    pub async fn write_blocks(
        &mut self,
        block_idx: u32,
        blocks: &[DataBlock],
    ) -> Result<(), Error> {
        let mut remaining = blocks;
        let mut current = block_idx;
        while !remaining.is_empty() {
            let batch_blocks = core::cmp::min(remaining.len(), MAX_DMA_BLOCKS_PER_BATCH);
            let (batch, rest) = remaining.split_at(batch_blocks);
            let buffer = unsafe {
                core::slice::from_raw_parts(batch.as_ptr().cast::<u8>(), batch.len() * BLOCK_LEN)
            };
            let address = self.block_address(current);
            let command = if batch.len() == 1 {
                CMD24_WRITE_BLOCK
            } else {
                CMD25_WRITE_MULTIPLE_BLOCKS
            };
            self.sdmmc
                .transfer_write(command, address, buffer, batch.len())
                .await?;
            self.wait_for_programming_done().await?;
            current = current.saturating_add(batch.len() as u32);
            remaining = rest;
        }
        Ok(())
    }

    async fn wait_for_programming_done(&mut self) -> Result<(), Error> {
        let deadline = deadline_counter(PROGRAMMING_TIMEOUT_MS);
        loop {
            let status = self.read_status().await?;
            match status.state() {
                Ok(response::State::Tran) => return Ok(()),
                Ok(response::State::Data | response::State::Rcv | response::State::Prg) => {}
                Ok(_) => return Err(Error::UnexpectedResponse),
                Err(_) => return Err(Error::UnexpectedResponse),
            }
            if timeout_elapsed(deadline) {
                return Err(Error::ProgrammingTimeout);
            }
        }
    }

    async fn read_status(&mut self) -> Result<response::CardStatus, Error> {
        self.sdmmc
            .send_command(CMD13_SEND_STATUS, (self.info.rca as u32) << 16)
            .await?;
        Ok(R1::new_with_raw_value(self.sdmmc.read_u32_response()))
    }

    fn block_address(&self, block_idx: u32) -> u32 {
        match self.info.card_type {
            CardType::SD1 | CardType::SD2 => block_idx * BLOCK_LEN as u32,
            CardType::SdhcSdxc => block_idx,
        }
    }
}

async fn acquire_card<T: super::Instance>(sdmmc: &mut Sdmmc<'_, T>) -> Result<SdCardInfo, Error> {
    if !sdmmc.read_present_state().card_inserted() {
        return Err(Error::NoCard);
    }

    sdmmc.send_command(CMD0_GO_IDLE_MODE, 0).await?;

    let cmd8_arg = argument::Cmd8::ZERO
        .with_voltage_supplied(argument::VoltageSuppliedSelect::_2_7To3_6V)
        .with_check_pattern(0xAA)
        .raw_value();
    let mut cmd8_supported = false;
    let mut last_cmd8_err = None;
    for _ in 0..=CMD8_RETRIES {
        match sdmmc.send_command(CMD8_SEND_IF_COND, cmd8_arg).await {
            Ok(_) => {
                let r7 = R7::new_with_raw_value(sdmmc.read_u32_response());
                if !r7
                    .voltage_accepted()
                    .is_ok_and(|voltage| voltage == argument::VoltageSuppliedSelect::_2_7To3_6V)
                {
                    return Err(Error::UnsupportedVoltage);
                }
                if r7.echo_check_pattern() != 0xAA {
                    return Err(Error::UnsupportedCardVersion);
                }
                cmd8_supported = true;
                break;
            }
            Err(Error::ResponseError(bits)) if bits.timeout() => {
                last_cmd8_err = Some(Error::ResponseError(bits));
            }
            Err(err) => return Err(err),
        }
    }
    if !cmd8_supported && last_cmd8_err.is_none() {
        return Err(Error::UnsupportedCardVersion);
    }

    let mut r3 = embedded_sdmmc::sdcard::response::R3::ZERO;
    let deadline = deadline_counter(CARD_INIT_TIMEOUT_MS);
    loop {
        send_acmd(
            sdmmc,
            ACMD41_SEND_OP_COND,
            argument::Acmd41::builder()
                .with_host_capacity_support(if cmd8_supported {
                    argument::HostCapacitySupport::SdhcOrSdxc
                } else {
                    argument::HostCapacitySupport::SdscOnly
                })
                .with_fast_boot(false)
                .with_xpc(argument::PowerControl::MaximumPerformance)
                .with_s18r(false)
                .with_ocr(VOLTAGE_LEVEL_CAPABILITIES)
                .build()
                .raw_value(),
            0,
        )
        .await?;
        r3 = embedded_sdmmc::sdcard::response::R3::new_with_raw_value(sdmmc.read_u32_response());
        if r3.initialization_complete() {
            break;
        }
        if timeout_elapsed(deadline) {
            return Err(Error::CardInitTimeout);
        }
    }

    let card_type = if r3.card_capacity_status() {
        CardType::SdhcSdxc
    } else if cmd8_supported {
        CardType::SD2
    } else {
        CardType::SD1
    };

    sdmmc.send_command(CMD2_ALL_SEND_CID, 0).await?;
    let cid_raw = sdmmc.read_u128_response();
    let cid = embedded_sdmmc::sdcard::cid::Cid::new_with_raw_value(u128::from_be_bytes(cid_raw));

    sdmmc.send_command(CMD3_SEND_RELATIVE_ADDR, 0).await?;
    let rca = R6::new_with_raw_value(sdmmc.read_u32_response()).rca();

    sdmmc
        .send_command(CMD9_SEND_CSD, (rca as u32) << 16)
        .await?;
    let csd_raw = sdmmc.read_u128_response();
    let csd = Csd::new_unchecked(&csd_raw).map_err(map_csd_error)?;

    sdmmc
        .send_command(CMD7_SELECT_SD_CARD, (rca as u32) << 16)
        .await?;
    let card_status = read_status(sdmmc, rca).await?;
    match card_status.state() {
        Ok(response::State::Tran) => {}
        Ok(_) | Err(_) => return Err(Error::UnexpectedResponse),
    }

    let supports_4bit = read_scr_supports_4bit(sdmmc, rca).await?;
    if sdmmc.bus_mode() == BusMode::FourBit && supports_4bit {
        send_acmd(
            sdmmc,
            ACMD6_SET_BUS_WIDTH,
            argument::Acmd6::builder()
                .with_bus_width(argument::BusWidth::_4bits)
                .build()
                .raw_value(),
            rca,
        )
        .await?;
        sdmmc.set_card_bus_width(BusMode::FourBit);
    }

    if !matches!(card_type, CardType::SdhcSdxc) {
        sdmmc
            .send_command(CMD16_SET_BLOCKLEN, BLOCK_LEN as u32)
            .await?;
    }

    sdmmc.switch_to_normal_transfer_clock();

    Ok(SdCardInfo {
        card_type,
        rca,
        cid,
        csd,
    })
}

fn deadline_counter(timeout_ms: u64) -> u64 {
    let clocks = clocks::get();
    let counter = GlobalTimerCounter::new(clocks.arm_clocks()).read_timer();
    let ticks =
        ((clocks.arm_clocks().cpu_3x2x_clk().raw() as u128) * (timeout_ms as u128)).div_ceil(1000);
    counter.saturating_add(ticks.min(u64::MAX as u128) as u64)
}

fn timeout_elapsed(deadline: u64) -> bool {
    let now = GlobalTimerCounter::new(clocks::get().arm_clocks()).read_timer();
    now >= deadline
}

fn map_csd_error(_: InvalidCsdStructureFieldError) -> Error {
    Error::UnexpectedResponse
}

async fn read_scr_supports_4bit<T: super::Instance>(
    sdmmc: &mut Sdmmc<'_, T>,
    rca: u16,
) -> Result<bool, Error> {
    let mut scr = [0u8; SCR_LEN];
    sdmmc
        .send_command(CMD16_SET_BLOCKLEN, SCR_LEN as u32)
        .await?;
    send_acmd_read(sdmmc, ACMD51_SEND_SCR, 0, rca, &mut scr, 1, SCR_LEN).await?;
    Ok(scr[1] & SCR_BUS_WIDTH_4BIT_MASK != 0)
}

async fn read_status<T: super::Instance>(
    sdmmc: &mut Sdmmc<'_, T>,
    rca: u16,
) -> Result<response::CardStatus, Error> {
    sdmmc
        .send_command(CMD13_SEND_STATUS, (rca as u32) << 16)
        .await?;
    Ok(R1::new_with_raw_value(sdmmc.read_u32_response()))
}

async fn send_acmd<T: super::Instance>(
    sdmmc: &mut Sdmmc<'_, T>,
    acmd: zynq7000::sdio::CommandRegister,
    arg: u32,
    rca: u16,
) -> Result<super::StatusWrapper, Error> {
    sdmmc
        .send_command(CMD55_APP_CMD, (rca as u32) << 16)
        .await?;
    let r1 = R1::new_with_raw_value(sdmmc.read_u32_response());
    if !r1.app_cmd() {
        return Err(Error::UnexpectedResponse);
    }
    sdmmc.send_command(acmd, arg).await
}

async fn send_acmd_read<T: super::Instance>(
    sdmmc: &mut Sdmmc<'_, T>,
    acmd: zynq7000::sdio::CommandRegister,
    arg: u32,
    rca: u16,
    buffer: &mut [u8],
    blocks: usize,
    block_len: usize,
) -> Result<(), Error> {
    sdmmc
        .send_command(CMD55_APP_CMD, (rca as u32) << 16)
        .await?;
    let r1 = R1::new_with_raw_value(sdmmc.read_u32_response());
    if !r1.app_cmd() {
        return Err(Error::UnexpectedResponse);
    }
    sdmmc
        .transfer_read_with_block_len(acmd, arg, buffer, blocks, block_len)
        .await
}
