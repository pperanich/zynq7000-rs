macro_rules! impl_ttc {
    ($type:ident, $regs:expr) => {
        impl crate::ttc::SealedInstance for super::peripherals::$type {
            fn regs() -> super::pac::ttc::MmioRegisters<'static> {
                $regs
            }
        }

        impl crate::ttc::Instance for super::peripherals::$type {}
    };
}

impl_ttc!(TTC0, unsafe {
    super::pac::ttc::Registers::new_mmio_fixed_0()
});
impl_ttc!(TTC1, unsafe {
    super::pac::ttc::Registers::new_mmio_fixed_1()
});
