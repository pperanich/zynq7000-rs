#![allow(private_interfaces)]

macro_rules! impl_sdmmc {
    ($type:ident, $id:expr, $regs:expr, $state:expr, $irq:ident) => {
        impl crate::sdmmc::SealedInstance for super::peripherals::$type {
            fn id() -> crate::sdmmc::SdioId {
                $id
            }

            fn regs() -> super::pac::sdio::MmioRegisters<'static> {
                $regs
            }

            fn state() -> &'static crate::sdmmc::State {
                $state
            }
        }

        impl crate::sdmmc::Instance for super::peripherals::$type {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

impl_sdmmc!(
    SDIO0,
    crate::sdmmc::SdioId::Sdio0,
    unsafe { super::pac::sdio::Registers::new_mmio_fixed_0() },
    &crate::sdmmc::SDMMC_STATES[0],
    Sdio0
);
impl_sdmmc!(
    SDIO1,
    crate::sdmmc::SdioId::Sdio1,
    unsafe { super::pac::sdio::Registers::new_mmio_fixed_1() },
    &crate::sdmmc::SDMMC_STATES[1],
    Sdio1
);

macro_rules! impl_route_1bit {
    ($sdio:ty, $group:ident, $clk:ident, $cmd:ident, $d0:ident) => {
        pub enum $group {}
        impl crate::sdmmc::sealed::RouteGroup for $group {}

        impl crate::sdmmc::sealed::ClockPin<$sdio> for super::peripherals::$clk {
            type RouteGroup = $group;
            fn mux_config() -> crate::gpio::MuxConfig {
                crate::sdmmc::MUX_CONF
            }
        }
        impl crate::sdmmc::ClockPin<$sdio> for super::peripherals::$clk {}

        impl crate::sdmmc::sealed::CommandPin<$sdio> for super::peripherals::$cmd {
            type RouteGroup = $group;
            fn mux_config() -> crate::gpio::MuxConfig {
                crate::sdmmc::MUX_CONF
            }
        }
        impl crate::sdmmc::CommandPin<$sdio> for super::peripherals::$cmd {}

        impl crate::sdmmc::sealed::Data0Pin<$sdio> for super::peripherals::$d0 {
            type RouteGroup = $group;
            fn mux_config() -> crate::gpio::MuxConfig {
                crate::sdmmc::MUX_CONF
            }
        }
        impl crate::sdmmc::Data0Pin<$sdio> for super::peripherals::$d0 {}

        impl
            crate::sdmmc::sealed::Bus1Bit<
                $sdio,
                super::peripherals::$clk,
                super::peripherals::$cmd,
                super::peripherals::$d0,
            > for ()
        {
        }
        impl
            crate::sdmmc::Bus1Bit<
                $sdio,
                super::peripherals::$clk,
                super::peripherals::$cmd,
                super::peripherals::$d0,
            > for ()
        {
        }
    };
}

macro_rules! impl_route_4bit {
    ($sdio:ty, $group:ident, $clk:ident, $cmd:ident, $d0:ident, $d1:ident, $d2:ident, $d3:ident) => {
        impl_route_1bit!($sdio, $group, $clk, $cmd, $d0);

        impl crate::sdmmc::sealed::Data1Pin<$sdio> for super::peripherals::$d1 {
            type RouteGroup = $group;
            fn mux_config() -> crate::gpio::MuxConfig {
                crate::sdmmc::MUX_CONF
            }
        }
        impl crate::sdmmc::Data1Pin<$sdio> for super::peripherals::$d1 {}

        impl crate::sdmmc::sealed::Data2Pin<$sdio> for super::peripherals::$d2 {
            type RouteGroup = $group;
            fn mux_config() -> crate::gpio::MuxConfig {
                crate::sdmmc::MUX_CONF
            }
        }
        impl crate::sdmmc::Data2Pin<$sdio> for super::peripherals::$d2 {}

        impl crate::sdmmc::sealed::Data3Pin<$sdio> for super::peripherals::$d3 {
            type RouteGroup = $group;
            fn mux_config() -> crate::gpio::MuxConfig {
                crate::sdmmc::MUX_CONF
            }
        }
        impl crate::sdmmc::Data3Pin<$sdio> for super::peripherals::$d3 {}

        impl
            crate::sdmmc::sealed::Bus4Bit<
                $sdio,
                super::peripherals::$clk,
                super::peripherals::$cmd,
                super::peripherals::$d0,
                super::peripherals::$d1,
                super::peripherals::$d2,
                super::peripherals::$d3,
            > for ()
        {
        }
        impl
            crate::sdmmc::Bus4Bit<
                $sdio,
                super::peripherals::$clk,
                super::peripherals::$cmd,
                super::peripherals::$d0,
                super::peripherals::$d1,
                super::peripherals::$d2,
                super::peripherals::$d3,
            > for ()
        {
        }
    };
}

#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_route_4bit!(
    super::peripherals::SDIO0,
    Sdio0Route0,
    MIO16,
    MIO17,
    MIO18,
    MIO19,
    MIO20,
    MIO21
);
impl_route_4bit!(
    super::peripherals::SDIO0,
    Sdio0Route1,
    MIO28,
    MIO29,
    MIO30,
    MIO31,
    MIO32,
    MIO33
);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_route_4bit!(
    super::peripherals::SDIO0,
    Sdio0Route2,
    MIO40,
    MIO41,
    MIO42,
    MIO43,
    MIO44,
    MIO45
);

impl_route_4bit!(
    super::peripherals::SDIO1,
    Sdio1Route0,
    MIO12,
    MIO11,
    MIO10,
    MIO13,
    MIO14,
    MIO15
);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_route_4bit!(
    super::peripherals::SDIO1,
    Sdio1Route1,
    MIO24,
    MIO23,
    MIO22,
    MIO25,
    MIO26,
    MIO27
);
impl_route_4bit!(
    super::peripherals::SDIO1,
    Sdio1Route2,
    MIO36,
    MIO35,
    MIO34,
    MIO37,
    MIO38,
    MIO39
);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_route_4bit!(
    super::peripherals::SDIO1,
    Sdio1Route3,
    MIO48,
    MIO47,
    MIO46,
    MIO49,
    MIO50,
    MIO51
);
