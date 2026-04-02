macro_rules! impl_usb {
    ($type:ident, $id:expr, $irq:ident) => {
        impl crate::usb::SealedInstance for super::peripherals::$type {
            fn id() -> crate::usb::UsbId {
                $id
            }
        }

        impl crate::usb::Instance for super::peripherals::$type {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

impl_usb!(USB0, crate::usb::UsbId::Usb0, Usb0);
impl_usb!(USB1, crate::usb::UsbId::Usb1, Usb1);
