#![allow(private_interfaces)]

impl crate::qspi::SealedInstance for super::peripherals::QSPI {
    fn regs() -> super::pac::qspi::MmioRegisters<'static> {
        unsafe { super::pac::qspi::Registers::new_mmio_fixed() }
    }
}

impl crate::qspi::Instance for super::peripherals::QSPI {}

macro_rules! impl_qspi_pin {
    ($trait_name:ident, $pin:ident) => {
        impl crate::qspi::sealed::$trait_name<super::peripherals::QSPI>
            for super::peripherals::$pin
        {
            fn mux_config() -> crate::gpio::MuxConfig {
                crate::gpio::MuxConfig::new(
                    true,
                    false,
                    arbitrary_int::u2::new(0),
                    arbitrary_int::u3::new(0),
                )
            }
        }
        impl crate::qspi::$trait_name<super::peripherals::QSPI> for super::peripherals::$pin {}
    };
}

impl_qspi_pin!(ChipSelect0Pin, MIO1);
impl_qspi_pin!(Io0Pin, MIO2);
impl_qspi_pin!(Io1Pin, MIO3);
impl_qspi_pin!(Io2Pin, MIO4);
impl_qspi_pin!(Io3Pin, MIO5);
impl_qspi_pin!(ClockPin, MIO6);
impl_qspi_pin!(ChipSelect1Pin, MIO0);
impl_qspi_pin!(FeedbackClockPin, MIO8);
