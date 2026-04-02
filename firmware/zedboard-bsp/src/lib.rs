#![no_std]

pub const PS_CLOCK_FREQUENCY: zynq7000_hal::time::Hertz =
    zynq7000_hal::time::Hertz::from_raw(33_333_300);

pub mod ddrc_config_autogen;
pub mod ddriob_config_autogen;
pub mod phy_marvell;
pub mod qspi_spansion;
