#![allow(private_interfaces)]

/// UART MIO routing metadata derived from UG585's MIO-at-a-Glance figure.
///
/// The TRM documents legal UART signal routing as fixed MIO groups. The routing must be kept as a
/// group and may not be split across different MIO groups or between MIO and EMIO.
///
/// At present all legal UART MIO routes appear to use the same SLCR slow-peripheral mux select,
/// so the per-pin metadata here primarily captures legality and package availability rather than
/// distinct mux bit patterns.

macro_rules! impl_uart {
    ($type:ident, $id:expr, $regs:expr, $state:expr, $irq:ident) => {
        impl crate::uart::SealedInstance for super::peripherals::$type {
            fn id() -> crate::uart::UartId {
                $id
            }

            fn regs() -> super::pac::uart::MmioRegisters<'static> {
                $regs
            }

            fn state() -> &'static crate::uart::State {
                $state
            }
        }

        impl crate::uart::Instance for super::peripherals::$type {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

impl_uart!(
    UART0,
    crate::uart::UartId::Uart0,
    unsafe { super::pac::uart::Registers::new_mmio_fixed_0() },
    &crate::uart::UART_STATES[0],
    Uart0
);
impl_uart!(
    UART1,
    crate::uart::UartId::Uart1,
    unsafe { super::pac::uart::Registers::new_mmio_fixed_1() },
    &crate::uart::UART_STATES[1],
    Uart1
);

macro_rules! impl_uart_pins {
    ($uart:ty, ($( [$(#[$meta:meta], )? $group:ident, $tx:ident, $tx_mio:literal, $rx:ident, $rx_mio:literal] ),+ $(,)?)) => {
        $(
            $(#[$meta])?
            pub enum $group {}
            $(#[$meta])?
            impl crate::uart::sealed::RouteGroup for $group {}

            $(#[$meta])?
            impl crate::uart::sealed::TxPin<$uart> for super::peripherals::$tx {
                type RouteGroup = $group;

                fn metadata() -> crate::uart::sealed::PinMetadata {
                    crate::uart::sealed::PinMetadata {
                        mio: $tx_mio,
                        direction: crate::uart::sealed::PinDirection::Tx,
                        route_id: $tx_mio / 4,
                        mux_config: crate::gpio::MuxConfig::new_with_l3(arbitrary_int::u3::new(0b111)),
                    }
                }
            }
            $(#[$meta])?
            impl crate::uart::TxPin<$uart> for super::peripherals::$tx {}
            $(#[$meta])?
            impl crate::uart::sealed::RxPin<$uart> for super::peripherals::$rx {
                type RouteGroup = $group;

                fn metadata() -> crate::uart::sealed::PinMetadata {
                    crate::uart::sealed::PinMetadata {
                        mio: $rx_mio,
                        direction: crate::uart::sealed::PinDirection::Rx,
                        route_id: $tx_mio / 4,
                        mux_config: crate::gpio::MuxConfig::new_with_l3(arbitrary_int::u3::new(0b111)),
                    }
                }
            }
            $(#[$meta])?
            impl crate::uart::RxPin<$uart> for super::peripherals::$rx {}
        )+
    };
}

impl_uart_pins!(
    super::peripherals::UART0,
    (
        [Route0, MIO11, 11, MIO10, 10],
        [Route1, MIO15, 15, MIO14, 14],
        // These MIO pins are unavailable on 7z010/7z007s CLG225 devices, which expose only
        // MIO[31:0]; see UG585 "MIO Pins in 7z007s and 7z010 CLG225 Devices".
        [#[cfg(not(feature = "7z010-7z007s-clg225"))], Route2, MIO19, 19, MIO18, 18],
        [#[cfg(not(feature = "7z010-7z007s-clg225"))], Route3, MIO23, 23, MIO22, 22],
        [#[cfg(not(feature = "7z010-7z007s-clg225"))], Route4, MIO27, 27, MIO26, 26],
        [Route5, MIO31, 31, MIO30, 30],
        [Route6, MIO35, 35, MIO34, 34],
        [Route7, MIO39, 39, MIO38, 38],
        [#[cfg(not(feature = "7z010-7z007s-clg225"))], Route8, MIO43, 43, MIO42, 42],
        [#[cfg(not(feature = "7z010-7z007s-clg225"))], Route9, MIO47, 47, MIO46, 46],
        [#[cfg(not(feature = "7z010-7z007s-clg225"))], Route10, MIO51, 51, MIO50, 50],
    )
);

impl_uart_pins!(
    super::peripherals::UART1,
    (
        [Route11, MIO8, 8, MIO9, 9],
        [Route12, MIO12, 12, MIO13, 13],
        // These MIO pins are unavailable on 7z010/7z007s CLG225 devices, which expose only
        // MIO[31:0]; see UG585 "MIO Pins in 7z007s and 7z010 CLG225 Devices".
        [#[cfg(not(feature = "7z010-7z007s-clg225"))], Route13, MIO16, 16, MIO17, 17],
        [#[cfg(not(feature = "7z010-7z007s-clg225"))], Route14, MIO20, 20, MIO21, 21],
        [#[cfg(not(feature = "7z010-7z007s-clg225"))], Route15, MIO24, 24, MIO25, 25],
        [Route16, MIO28, 28, MIO29, 29],
        [Route17, MIO32, 32, MIO33, 33],
        [Route18, MIO36, 36, MIO37, 37],
        [#[cfg(not(feature = "7z010-7z007s-clg225"))], Route19, MIO40, 40, MIO41, 41],
        [#[cfg(not(feature = "7z010-7z007s-clg225"))], Route20, MIO44, 44, MIO45, 45],
        [Route21, MIO48, 48, MIO49, 49],
        [Route22, MIO52, 52, MIO53, 53],
    )
);
