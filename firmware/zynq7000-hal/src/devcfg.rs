//! Device configuration and PCAP helpers.

use crate::cache::{CACHE_LINE_SIZE, clean_data_cache_range};

/// Runtime configuration for a PCAP bitstream transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PcapConfig {
    /// Toggle `PROG_B` and wait for `pcfg_init` before starting the transfer.
    pub init_pl: bool,
    /// Enable the PCAP rate control bit before the transfer.
    pub rate_enable: bool,
}

impl Default for PcapConfig {
    fn default() -> Self {
        Self {
            init_pl: true,
            rate_enable: false,
        }
    }
}

/// Successful blocking PCAP transfer result.
#[derive(Debug, Clone, Copy)]
pub struct BitstreamLoadStatus {
    /// Final devcfg interrupt-status value observed at completion.
    pub interrupt_status: zynq7000::devcfg::Interrupt,
}

/// Blocking PCAP transfer error.
#[derive(Debug, thiserror::Error)]
pub enum BitstreamLoadError {
    #[error("unaligned bitstream address: {0:#x}")]
    UnalignedAddress(usize),
    #[error("bitstream length must be a multiple of 4 bytes: {0}")]
    UnalignedWordLen(usize),
    #[error("DMA illegal command")]
    DmaIllegalCommand,
    #[error("DMA queue overflow")]
    DmaQueueOverflow,
    #[error("PCAP to DMA transfer length mismatch")]
    TransferLengthMismatch,
    #[error("AXI write timeout")]
    AxiWriteTimeout,
    #[error("AXI write response error")]
    AxiWriteResponseError,
    #[error("AXI read timeout")]
    AxiReadTimeout,
    #[error("AXI read response error")]
    AxiReadResponseError,
    #[error("HMAC error during configuration")]
    HmacError,
    #[error("SEU error during configuration")]
    SeuError,
    #[error("PL power loss / POR during configuration")]
    PlPowerLoss,
    #[error("PL configuration controller is held in reset")]
    PlConfigurationControllerReset,
}

/// Legacy alignment-only error retained for the compatibility helper.
#[derive(Debug, thiserror::Error)]
pub enum UnalignedAddrError {
    #[error("unaligned address: {0}")]
    Address(usize),
    #[error("bitstream length must be a multiple of 4 bytes: {0}")]
    Length(usize),
    #[error(transparent)]
    Transfer(#[from] BitstreamLoadError),
}

/// Typed devcfg driver.
pub struct Devcfg {
    regs: zynq7000::devcfg::MmioRegisters<'static>,
}

fn cache_maintenance_range(addr: usize, len: usize) -> (u32, usize) {
    let start = addr & !(CACHE_LINE_SIZE - 1);
    let end = (addr + len + CACHE_LINE_SIZE - 1) & !(CACHE_LINE_SIZE - 1);
    (start as u32, end - start)
}

fn validate_bitstream(bitstream: &[u8]) -> Result<(), BitstreamLoadError> {
    if bitstream.is_empty() {
        return Ok(());
    }
    if !(bitstream.as_ptr() as usize).is_multiple_of(64) {
        return Err(BitstreamLoadError::UnalignedAddress(
            bitstream.as_ptr() as usize
        ));
    }
    if !bitstream.len().is_multiple_of(4) {
        return Err(BitstreamLoadError::UnalignedWordLen(bitstream.len()));
    }
    Ok(())
}

impl Devcfg {
    /// Create a driver from the fixed devcfg MMIO block.
    ///
    /// # Safety
    ///
    /// The returned driver aliases the global devcfg register block.
    pub unsafe fn steal() -> Self {
        Self::new(unsafe { zynq7000::devcfg::Registers::new_mmio_fixed() })
    }

    /// Create a driver from a devcfg MMIO handle.
    pub const fn new(regs: zynq7000::devcfg::MmioRegisters<'static>) -> Self {
        Self { regs }
    }

    /// Configure devcfg for a non-secure PCAP transfer.
    pub fn configure_pcap_non_secure(&mut self, config: PcapConfig) {
        self.regs.modify_control(|mut val| {
            val.set_config_access_select(zynq7000::devcfg::PlConfigAccess::ConfigAccessPort);
            val.set_access_port_select(zynq7000::devcfg::ConfigAccessPortSelect::Pcap);
            val.set_pcap_rate_enable(config.rate_enable);
            val
        });
        self.regs
            .write_interrupt_status(zynq7000::devcfg::Interrupt::ack_all());
        if config.init_pl {
            self.regs.modify_control(|mut val| {
                val.set_prog_b_bit(true);
                val
            });
            self.regs.modify_control(|mut val| {
                val.set_prog_b_bit(false);
                val
            });
            while self.regs.read_status().pcfg_init() {}
            self.regs.modify_control(|mut val| {
                val.set_prog_b_bit(true);
                val
            });
            self.regs
                .write_interrupt_status(zynq7000::devcfg::Interrupt::ack_pl_programming_done());
        }
        while !self.regs.read_status().pcfg_init() {}
        if !config.init_pl {
            while self.regs.read_status().dma_command_queue_full() {}
        }
        self.regs.modify_misc_control(|mut val| {
            val.set_loopback(false);
            val
        });
    }

    /// Load a bitstream through PCAP in blocking mode.
    pub fn load_bitstream_non_secure(
        &mut self,
        config: PcapConfig,
        bitstream: &[u8],
    ) -> Result<BitstreamLoadStatus, BitstreamLoadError> {
        validate_bitstream(bitstream)?;
        if bitstream.is_empty() {
            return Ok(BitstreamLoadStatus {
                interrupt_status: zynq7000::devcfg::Interrupt::ZERO,
            });
        }

        self.configure_pcap_non_secure(config);
        let (addr, len) = cache_maintenance_range(bitstream.as_ptr() as usize, bitstream.len());
        clean_data_cache_range(addr, len).expect("PCAP DMA cache preparation failed");

        // Setting the two LSBs of the source and destination address to `01` marks the final
        // DMA command of the transfer.
        self.regs
            .write_dma_source_addr(bitstream.as_ptr() as u32 | 0b01);
        self.regs.write_dma_dest_addr(0xFFFF_FFFF);
        self.regs.write_dma_source_len((bitstream.len() / 4) as u32);
        self.regs.write_dma_dest_len((bitstream.len() / 4) as u32);

        loop {
            let isr = self.regs.read_interrupt_status();
            if isr.dma_illegal_command() {
                return Err(BitstreamLoadError::DmaIllegalCommand);
            }
            if isr.dma_queue_overflow() {
                return Err(BitstreamLoadError::DmaQueueOverflow);
            }
            if isr.inconsistent_pcap_to_dma_transfer_len() {
                return Err(BitstreamLoadError::TransferLengthMismatch);
            }
            if isr.axi_write_timeout() {
                return Err(BitstreamLoadError::AxiWriteTimeout);
            }
            if isr.axi_write_response_error() {
                return Err(BitstreamLoadError::AxiWriteResponseError);
            }
            if isr.axi_read_timeout() {
                return Err(BitstreamLoadError::AxiReadTimeout);
            }
            if isr.axi_read_response_error() {
                return Err(BitstreamLoadError::AxiReadResponseError);
            }
            if isr.hamc_error() {
                return Err(BitstreamLoadError::HmacError);
            }
            if isr.seu_error() {
                return Err(BitstreamLoadError::SeuError);
            }
            if isr.pl_power_loss_por_b_low() {
                return Err(BitstreamLoadError::PlPowerLoss);
            }
            if isr.pl_config_controller_under_reset() {
                return Err(BitstreamLoadError::PlConfigurationControllerReset);
            }
            if isr.dma_done() && isr.pl_programming_done() {
                return Ok(BitstreamLoadStatus {
                    interrupt_status: isr,
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn empty_bitstream_is_valid_even_if_pointer_is_not_dma_aligned() {
        let empty = &[] as &[u8];
        assert!(validate_bitstream(empty).is_ok());
    }

    #[test]
    fn cache_range_rounds_to_cache_line_boundaries() {
        assert_eq!(cache_maintenance_range(0x1043, 1), (0x1040, 32));
        assert_eq!(cache_maintenance_range(0x1043, 64), (0x1040, 96));
    }
}

/// Configures the bitstream using the PCAP interface in non-secure mode.
///
/// Blocking function which only returns when the bitstream configuration is complete.
pub fn configure_bitstream_non_secure(
    init_pl: bool,
    bitstream: &[u8],
) -> Result<(), UnalignedAddrError> {
    let mut devcfg = unsafe { Devcfg::steal() };
    devcfg
        .load_bitstream_non_secure(
            PcapConfig {
                init_pl,
                rate_enable: false,
            },
            bitstream,
        )
        .map(|_| ())
        .map_err(|err| match err {
            BitstreamLoadError::UnalignedAddress(addr) => UnalignedAddrError::Address(addr),
            BitstreamLoadError::UnalignedWordLen(len) => UnalignedAddrError::Length(len),
            other => UnalignedAddrError::Transfer(other),
        })
}
