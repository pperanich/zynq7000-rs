macro_rules! impl_mio_pin {
    ($name:ident, $offset:literal, $input_capable:ident) => {
        impl crate::gpio::SealedPin for super::peripherals::$name {
            fn offset(&self) -> u8 {
                $offset
            }
        }

        impl From<super::peripherals::$name> for crate::gpio::AnyPin {
            fn from(_: super::peripherals::$name) -> Self {
                crate::gpio::AnyPin::new($offset)
            }
        }

        impl crate::gpio::Pin for super::peripherals::$name {}

        if_input_capable!($input_capable, super::peripherals::$name, $offset);
    };
}

macro_rules! if_input_capable {
    (true, $pin:ty, $offset:literal) => {
        impl crate::gpio::sealed::InputPin for $pin {}
        impl crate::gpio::InputPin for $pin {}
    };
    (false, $pin:ty, $offset:literal) => {};
}

impl_mio_pin!(MIO0, 0, true);
impl_mio_pin!(MIO1, 1, true);
impl_mio_pin!(MIO2, 2, true);
impl_mio_pin!(MIO3, 3, true);
impl_mio_pin!(MIO4, 4, true);
impl_mio_pin!(MIO5, 5, true);
impl_mio_pin!(MIO6, 6, true);
impl_mio_pin!(MIO7, 7, false);
impl_mio_pin!(MIO8, 8, false);
impl_mio_pin!(MIO9, 9, true);
impl_mio_pin!(MIO10, 10, true);
impl_mio_pin!(MIO11, 11, true);
impl_mio_pin!(MIO12, 12, true);
impl_mio_pin!(MIO13, 13, true);
impl_mio_pin!(MIO14, 14, true);
impl_mio_pin!(MIO15, 15, true);
impl_mio_pin!(MIO16, 16, true);
impl_mio_pin!(MIO17, 17, true);
impl_mio_pin!(MIO18, 18, true);
impl_mio_pin!(MIO19, 19, true);
impl_mio_pin!(MIO20, 20, true);
impl_mio_pin!(MIO21, 21, true);
impl_mio_pin!(MIO22, 22, true);
impl_mio_pin!(MIO23, 23, true);
impl_mio_pin!(MIO24, 24, true);
impl_mio_pin!(MIO25, 25, true);
impl_mio_pin!(MIO26, 26, true);
impl_mio_pin!(MIO27, 27, true);
impl_mio_pin!(MIO28, 28, true);
impl_mio_pin!(MIO29, 29, true);
impl_mio_pin!(MIO30, 30, true);
impl_mio_pin!(MIO31, 31, true);
impl_mio_pin!(MIO32, 32, true);
impl_mio_pin!(MIO33, 33, true);
impl_mio_pin!(MIO34, 34, true);
impl_mio_pin!(MIO35, 35, true);
impl_mio_pin!(MIO36, 36, true);
impl_mio_pin!(MIO37, 37, true);
impl_mio_pin!(MIO38, 38, true);
impl_mio_pin!(MIO39, 39, true);
impl_mio_pin!(MIO40, 40, true);
impl_mio_pin!(MIO41, 41, true);
impl_mio_pin!(MIO42, 42, true);
impl_mio_pin!(MIO43, 43, true);
impl_mio_pin!(MIO44, 44, true);
impl_mio_pin!(MIO45, 45, true);
impl_mio_pin!(MIO46, 46, true);
impl_mio_pin!(MIO47, 47, true);
impl_mio_pin!(MIO48, 48, true);
impl_mio_pin!(MIO49, 49, true);
impl_mio_pin!(MIO50, 50, true);
impl_mio_pin!(MIO51, 51, true);
impl_mio_pin!(MIO52, 52, true);
impl_mio_pin!(MIO53, 53, true);
