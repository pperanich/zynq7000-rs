//! Chip inventory and instance-specific glue for `embassy-zynq7000`.
//!
//! This module is intentionally split in two layers:
//! - the generated token inventory exposed through [`Peripherals`] and [`peripherals`]
//! - small instance-specific glue modules that describe pin mappings and peripheral metadata
//!
//! Driver implementations should depend on the typed glue traits in the submodules rather than
//! hard-coding SoC-specific tables in the drivers themselves. This keeps the public driver
//! modules focused on Embassy-facing behavior while package- and instance-sensitive facts stay in
//! one internal place.

pub use zynq7000 as pac;

// Token inventory for the Zynq-7000 processing-system peripherals and MIO pins.
embassy_hal_internal::peripherals! {
    GICC,
    GICD,
    L2C,
    DDRC,
    UART0,
    UART1,
    SPI0,
    SPI1,
    I2C0,
    I2C1,
    GTC,
    GPIO,
    SLCR,
    TTC0,
    TTC1,
    USB0,
    USB1,
    ETH0,
    ETH1,
    QSPI,
    DEVCFG,
    XADC,
    SDIO0,
    SDIO1,
    MIO0,
    MIO1,
    MIO2,
    MIO3,
    MIO4,
    MIO5,
    MIO6,
    MIO7,
    MIO8,
    MIO9,
    MIO10,
    MIO11,
    MIO12,
    MIO13,
    MIO14,
    MIO15,
    MIO16,
    MIO17,
    MIO18,
    MIO19,
    MIO20,
    MIO21,
    MIO22,
    MIO23,
    MIO24,
    MIO25,
    MIO26,
    MIO27,
    MIO28,
    MIO29,
    MIO30,
    MIO31,
    MIO32,
    MIO33,
    MIO34,
    MIO35,
    MIO36,
    MIO37,
    MIO38,
    MIO39,
    MIO40,
    MIO41,
    MIO42,
    MIO43,
    MIO44,
    MIO45,
    MIO46,
    MIO47,
    MIO48,
    MIO49,
    MIO50,
    MIO51,
    MIO52,
    MIO53,
}

// Instance-specific metadata consumed by the Embassy-facing drivers.
mod i2c;
mod pins;
mod qspi;
mod sdmmc;
mod ttc;
mod uart;
mod usb;
