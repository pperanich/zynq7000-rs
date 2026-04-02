//! Blocking PS-XADC system monitor access.

use arbitrary_int::{u2, u4, u5};

/// DRP command opcodes for the PS-XADC interface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
enum DrpCommand {
    Noop = 0b0000,
    Read = 0b0011,
}

/// Supported on-chip XADC monitor channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    Temperature,
    VccInt,
    VccAux,
    VccBram,
}

impl Channel {
    const fn drp_addr(self) -> u8 {
        match self {
            Self::Temperature => 0x00,
            Self::VccInt => 0x01,
            Self::VccAux => 0x02,
            Self::VccBram => 0x06,
        }
    }
}

/// Raw 16-bit DRP register sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sample(u16);

impl Sample {
    /// Create a sample from a raw DRP register value.
    pub const fn from_raw(raw: u16) -> Self {
        Self(raw)
    }

    /// Return the raw DRP register value.
    pub const fn raw(self) -> u16 {
        self.0
    }

    /// Return the 12-bit ADC code encoded in bits `[15:4]`.
    pub const fn adc_code(self) -> u16 {
        self.0 >> 4
    }

    /// Convert a temperature sample to degrees Celsius.
    pub fn temperature_celsius(self) -> f32 {
        (self.adc_code() as f32 * 503.975 / 4096.0) - 273.15
    }

    /// Convert a voltage sample to volts.
    pub fn voltage_volts(self) -> f32 {
        self.adc_code() as f32 * 3.0 / 4096.0
    }
}

/// Blocking XADC read error.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("XADC command FIFO remained full")]
    CommandFifoFull,
    #[error("XADC data FIFO remained empty")]
    DataFifoEmpty,
}

/// Blocking XADC driver.
pub struct Xadc {
    regs: zynq7000::xadc::MmioRegisters<'static>,
}

impl Xadc {
    /// Create a driver from the fixed XADC MMIO block.
    ///
    /// # Safety
    ///
    /// The returned driver aliases the global XADC register block.
    pub unsafe fn steal() -> Self {
        Self::new(unsafe { zynq7000::xadc::Registers::new_mmio_fixed() })
    }

    /// Create a driver from an XADC MMIO handle.
    pub const fn new(regs: zynq7000::xadc::MmioRegisters<'static>) -> Self {
        Self { regs }
    }

    /// Reset and enable the PS-XADC interface using conservative defaults from XAPP1172.
    pub fn init(&mut self) {
        self.regs.modify_misc_control(|mut val| {
            val.set_reset(true);
            val
        });
        self.regs.modify_misc_control(|mut val| {
            val.set_reset(false);
            val
        });
        self.regs.write_config(
            zynq7000::xadc::Config::ZERO
                .with_enable(true)
                .with_cfifo_threshold(u4::new(0))
                .with_dfifo_threshold(u4::new(0))
                .with_write_data_active_edge(false)
                .with_read_data_active_edge(false)
                .with_tck_rate(u2::new(0b01))
                .with_inter_packet_gap(u5::new(20)),
        );
        self.regs
            .write_interrupt_status(zynq7000::xadc::InterruptStatus::ack_all());
    }

    /// Read a raw XADC monitor register.
    pub fn read_raw(&mut self, channel: Channel) -> Result<Sample, Error> {
        self.send_command(prepare_read_command(channel.drp_addr()))?;
        self.wait_command_fifo_empty()?;
        let _ = self.read_data_word()?;
        self.send_command(prepare_noop_command())?;
        self.wait_command_fifo_empty()?;
        let data = self.read_data_word()?;
        Ok(Sample::from_raw(data as u16))
    }

    /// Read the on-chip temperature sensor.
    pub fn read_temperature(&mut self) -> Result<Sample, Error> {
        self.read_raw(Channel::Temperature)
    }

    /// Read a supply-monitor channel.
    pub fn read_voltage(&mut self, channel: Channel) -> Result<Sample, Error> {
        self.read_raw(channel)
    }

    fn send_command(&mut self, command: u32) -> Result<(), Error> {
        if self.regs.read_misc_status().cfifo_full() {
            return Err(Error::CommandFifoFull);
        }
        self.regs
            .write_command_fifo(zynq7000::xadc::CommandFifo::new_with_raw_value(command));
        Ok(())
    }

    fn wait_command_fifo_empty(&self) -> Result<(), Error> {
        for _ in 0..10_000 {
            if self.regs.read_misc_status().cfifo_empty() {
                return Ok(());
            }
        }
        Err(Error::CommandFifoFull)
    }

    fn read_data_word(&mut self) -> Result<u32, Error> {
        for _ in 0..10_000 {
            if !self.regs.read_misc_status().dfifo_empty() {
                return Ok(self.regs.read_data_fifo().read_data());
            }
        }
        Err(Error::DataFifoEmpty)
    }
}

const fn prepare_read_command(addr: u8) -> u32 {
    ((DrpCommand::Read as u32) << 26) | ((addr as u32) << 16)
}

const fn prepare_noop_command() -> u32 {
    (DrpCommand::Noop as u32) << 26
}
