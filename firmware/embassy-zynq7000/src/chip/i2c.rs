#![allow(private_interfaces)]

macro_rules! impl_i2c {
    ($type:ident, $id:expr, $regs:expr, $irq:ident) => {
        impl crate::i2c::SealedInstance for super::peripherals::$type {
            fn id() -> crate::i2c::I2cId {
                $id
            }

            fn regs() -> super::pac::i2c::MmioRegisters<'static> {
                $regs
            }
        }

        impl crate::i2c::Instance for super::peripherals::$type {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

impl_i2c!(
    I2C0,
    crate::i2c::I2cId::I2c0,
    unsafe { super::pac::i2c::Registers::new_mmio_fixed_0() },
    I2c0
);
impl_i2c!(
    I2C1,
    crate::i2c::I2cId::I2c1,
    unsafe { super::pac::i2c::Registers::new_mmio_fixed_1() },
    I2c1
);

macro_rules! impl_i2c_pair {
    ($i2c:ty, $scl:ident, $sda:ident) => {
        impl crate::i2c::sealed::PinPair<$i2c, super::peripherals::$scl, super::peripherals::$sda>
            for ()
        {
        }
        impl crate::i2c::PinPair<$i2c, super::peripherals::$scl, super::peripherals::$sda> for () {}

        impl crate::i2c::sealed::SclPin<$i2c> for super::peripherals::$scl {
            fn mux_config() -> crate::gpio::MuxConfig {
                crate::gpio::MuxConfig::new_with_l3(arbitrary_int::u3::new(0b010))
            }
        }
        impl crate::i2c::SclPin<$i2c> for super::peripherals::$scl {}

        impl crate::i2c::sealed::SdaPin<$i2c> for super::peripherals::$sda {
            fn mux_config() -> crate::gpio::MuxConfig {
                crate::gpio::MuxConfig::new_with_l3(arbitrary_int::u3::new(0b010))
            }
        }
        impl crate::i2c::SdaPin<$i2c> for super::peripherals::$sda {}
    };
}

impl_i2c_pair!(super::peripherals::I2C0, MIO10, MIO11);
impl_i2c_pair!(super::peripherals::I2C0, MIO14, MIO15);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_i2c_pair!(super::peripherals::I2C0, MIO18, MIO19);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_i2c_pair!(super::peripherals::I2C0, MIO22, MIO23);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_i2c_pair!(super::peripherals::I2C0, MIO26, MIO27);
impl_i2c_pair!(super::peripherals::I2C0, MIO30, MIO31);
impl_i2c_pair!(super::peripherals::I2C0, MIO34, MIO35);
impl_i2c_pair!(super::peripherals::I2C0, MIO38, MIO39);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_i2c_pair!(super::peripherals::I2C0, MIO42, MIO43);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_i2c_pair!(super::peripherals::I2C0, MIO46, MIO47);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_i2c_pair!(super::peripherals::I2C0, MIO50, MIO51);

impl_i2c_pair!(super::peripherals::I2C1, MIO12, MIO13);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_i2c_pair!(super::peripherals::I2C1, MIO16, MIO17);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_i2c_pair!(super::peripherals::I2C1, MIO20, MIO21);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_i2c_pair!(super::peripherals::I2C1, MIO24, MIO25);
impl_i2c_pair!(super::peripherals::I2C1, MIO28, MIO29);
impl_i2c_pair!(super::peripherals::I2C1, MIO32, MIO33);
impl_i2c_pair!(super::peripherals::I2C1, MIO36, MIO37);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_i2c_pair!(super::peripherals::I2C1, MIO40, MIO41);
#[cfg(not(feature = "7z010-7z007s-clg225"))]
impl_i2c_pair!(super::peripherals::I2C1, MIO44, MIO45);
impl_i2c_pair!(super::peripherals::I2C1, MIO48, MIO49);
impl_i2c_pair!(super::peripherals::I2C1, MIO52, MIO53);
