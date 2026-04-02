use core::convert::Infallible;

use arbitrary_int::{u2, u3};
use embassy_hal_internal::{Peri, PeripheralType, impl_peripheral};
use zynq7000::{
    gpio::{MaskedOutput, MmioRegisters, Registers},
    slcr::mio::IoType,
};

pub use embedded_hal::digital::PinState;

use crate::slcr;

#[derive(Debug, thiserror::Error, Clone, Copy, PartialEq, Eq)]
#[error("MIO pin is output-only")]
pub struct PinIsOutputOnly;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PinMode {
    Disconnected,
    InputFloating,
    InputPullUp,
    OutputPushPull,
    OutputOpenDrain,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MuxConfig {
    l3: u3,
    l2: u2,
    l1: bool,
    l0: bool,
}

impl MuxConfig {
    pub(crate) const fn new(l0: bool, l1: bool, l2: u2, l3: u3) -> Self {
        Self { l3, l2, l1, l0 }
    }

    pub(crate) const fn new_with_l3(l3: u3) -> Self {
        Self::new(false, false, u2::new(0), l3)
    }

    pub(crate) const fn new_for_gpio() -> Self {
        Self::new(false, false, u2::new(0), u3::new(0))
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum PinOffset {
    Mio(usize),
}

pub(crate) struct LowLevelGpio {
    offset: PinOffset,
    regs: MmioRegisters<'static>,
}

impl LowLevelGpio {
    pub(crate) fn new(offset: PinOffset) -> Self {
        Self {
            offset,
            regs: unsafe { Registers::new_mmio_fixed() },
        }
    }

    pub(crate) fn configure_as_output_push_pull(&mut self, init_level: PinState) {
        crate::multicore::with_reconfiguration_lock(|| {
            let (offset, dirm, outen) = self.get_dirm_outen_regs_and_local_offset();
            self.reconfigure_slcr_mio_cfg(false, None, None, Some(MuxConfig::new_for_gpio()));
            self.write_state(init_level);
            let mut curr_dirm = unsafe { core::ptr::read_volatile(dirm) };
            curr_dirm |= 1 << offset;
            unsafe { core::ptr::write_volatile(dirm, curr_dirm) };
            let mut curr_outen = unsafe { core::ptr::read_volatile(outen) };
            curr_outen |= 1 << offset;
            unsafe { core::ptr::write_volatile(outen, curr_outen) };
        });
    }

    pub(crate) fn configure_as_output_open_drain(
        &mut self,
        init_level: PinState,
        with_internal_pullup: bool,
    ) {
        crate::multicore::with_reconfiguration_lock(|| {
            let (offset, dirm, outen) = self.get_dirm_outen_regs_and_local_offset();
            self.reconfigure_slcr_mio_cfg(
                false,
                Some(with_internal_pullup),
                None,
                Some(MuxConfig::new_for_gpio()),
            );
            self.write_state(init_level);
            let mut curr_dirm = unsafe { core::ptr::read_volatile(dirm) };
            curr_dirm |= 1 << offset;
            unsafe { core::ptr::write_volatile(dirm, curr_dirm) };
            let mut curr_outen = unsafe { core::ptr::read_volatile(outen) };
            if init_level == PinState::High {
                curr_outen &= !(1 << offset);
            } else {
                curr_outen |= 1 << offset;
            }
            unsafe { core::ptr::write_volatile(outen, curr_outen) };
        });
    }

    pub(crate) fn configure_as_disconnected(&mut self) {
        crate::multicore::with_reconfiguration_lock(|| {
            self.reconfigure_slcr_mio_cfg(true, None, None, None);
            self.configure_input_pin();
        });
    }

    pub(crate) fn configure_as_input_floating(&mut self) -> Result<(), PinIsOutputOnly> {
        let offset_raw = self.raw_offset();
        if offset_raw == 7 || offset_raw == 8 {
            return Err(PinIsOutputOnly);
        }
        crate::multicore::with_reconfiguration_lock(|| {
            self.reconfigure_slcr_mio_cfg(true, Some(false), None, Some(MuxConfig::new_for_gpio()));
            self.configure_input_pin();
        });
        Ok(())
    }

    pub(crate) fn configure_as_input_with_pull_up(&mut self) -> Result<(), PinIsOutputOnly> {
        let offset_raw = self.raw_offset();
        if offset_raw == 7 || offset_raw == 8 {
            return Err(PinIsOutputOnly);
        }
        crate::multicore::with_reconfiguration_lock(|| {
            self.reconfigure_slcr_mio_cfg(true, Some(true), None, Some(MuxConfig::new_for_gpio()));
            self.configure_input_pin();
        });
        Ok(())
    }

    pub(crate) fn configure_as_io_periph_pin(
        &mut self,
        mux_conf: MuxConfig,
        pullup: Option<bool>,
        io_type: Option<IoType>,
    ) {
        crate::multicore::with_reconfiguration_lock(|| {
            self.configure_input_pin();
            self.reconfigure_slcr_mio_cfg(false, pullup, io_type, Some(mux_conf));
        });
    }

    pub(crate) fn is_low(&self) -> bool {
        let (offset, in_reg) = self.get_data_in_reg_and_local_offset();
        let in_val = unsafe { core::ptr::read_volatile(in_reg) };
        ((in_val >> offset) & 1) == 0
    }

    pub(crate) fn is_high(&self) -> bool {
        !self.is_low()
    }

    pub(crate) fn is_set_low(&self) -> bool {
        let (offset, out_reg) = self.get_data_out_reg_and_local_offset();
        let out_val = unsafe { core::ptr::read_volatile(out_reg) };
        ((out_val >> offset) & 1) == 0
    }

    pub(crate) fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    pub(crate) fn enable_output_driver(&mut self) {
        crate::multicore::with_reconfiguration_lock(|| {
            let (offset, _, outen) = self.get_dirm_outen_regs_and_local_offset();
            let mut outen_reg = unsafe { core::ptr::read_volatile(outen) };
            outen_reg |= 1 << offset;
            unsafe { core::ptr::write_volatile(outen, outen_reg) };
        });
    }

    pub(crate) fn disable_output_driver(&mut self) {
        crate::multicore::with_reconfiguration_lock(|| {
            let (offset, _, outen) = self.get_dirm_outen_regs_and_local_offset();
            let mut outen_reg = unsafe { core::ptr::read_volatile(outen) };
            outen_reg &= !(1 << offset);
            unsafe { core::ptr::write_volatile(outen, outen_reg) };
        });
    }

    pub(crate) fn set_low(&mut self) {
        self.write_state(PinState::Low);
    }

    pub(crate) fn set_high(&mut self) {
        self.write_state(PinState::High);
    }

    fn raw_offset(&self) -> usize {
        match self.offset {
            PinOffset::Mio(offset) => offset,
        }
    }

    fn write_state(&mut self, level: PinState) {
        let (offset, masked_out_ptr) = self.get_masked_out_reg_and_local_offset();
        unsafe {
            core::ptr::write_volatile(
                masked_out_ptr,
                MaskedOutput::builder()
                    .with_mask(!(1 << offset))
                    .with_output((level as u16) << offset)
                    .build(),
            );
        }
    }

    fn reconfigure_slcr_mio_cfg(
        &mut self,
        tristate: bool,
        pullup: Option<bool>,
        io_type: Option<IoType>,
        mux_conf: Option<MuxConfig>,
    ) {
        let raw_offset = self.raw_offset();
        unsafe {
            slcr::with_unlocked(|slcr| {
                slcr.modify_mio_pins(raw_offset, |mut val| {
                    if let Some(pullup) = pullup {
                        val.set_pullup(pullup);
                    }
                    if let Some(io_type) = io_type {
                        val.set_io_type(io_type);
                    }
                    if let Some(mux_conf) = mux_conf {
                        val.set_l0_sel(mux_conf.l0);
                        val.set_l1_sel(mux_conf.l1);
                        val.set_l2_sel(mux_conf.l2);
                        val.set_l3_sel(mux_conf.l3);
                    }
                    val.set_tri_enable(tristate);
                    val
                })
                .unwrap();
            });
        }
    }

    fn configure_input_pin(&mut self) {
        let (offset, dirm, outen) = self.get_dirm_outen_regs_and_local_offset();
        let mut curr_dirm = unsafe { core::ptr::read_volatile(dirm) };
        curr_dirm &= !(1 << offset);
        unsafe { core::ptr::write_volatile(dirm, curr_dirm) };
        let mut curr_outen = unsafe { core::ptr::read_volatile(outen) };
        curr_outen &= !(1 << offset);
        unsafe { core::ptr::write_volatile(outen, curr_outen) };
    }

    fn get_data_in_reg_and_local_offset(&self) -> (usize, *mut u32) {
        match self.raw_offset() {
            0..=31 => (self.raw_offset(), self.regs.pointer_to_in_0()),
            32..=53 => (self.raw_offset() - 32, self.regs.pointer_to_in_1()),
            _ => panic!("invalid MIO pin offset"),
        }
    }

    fn get_data_out_reg_and_local_offset(&self) -> (usize, *mut u32) {
        match self.raw_offset() {
            0..=31 => (self.raw_offset(), self.regs.pointer_to_out_0()),
            32..=53 => (self.raw_offset() - 32, self.regs.pointer_to_out_1()),
            _ => panic!("invalid MIO pin offset"),
        }
    }

    fn get_dirm_outen_regs_and_local_offset(&self) -> (usize, *mut u32, *mut u32) {
        match self.raw_offset() {
            0..=31 => (
                self.raw_offset(),
                self.regs.bank_0_shared().pointer_to_dirm(),
                self.regs.bank_0_shared().pointer_to_out_en(),
            ),
            32..=53 => (
                self.raw_offset() - 32,
                self.regs.bank_1_shared().pointer_to_dirm(),
                self.regs.bank_1_shared().pointer_to_out_en(),
            ),
            _ => panic!("invalid MIO pin offset"),
        }
    }

    fn get_masked_out_reg_and_local_offset(&mut self) -> (usize, *mut MaskedOutput) {
        match self.raw_offset() {
            0..=15 => (self.raw_offset(), self.regs.pointer_to_masked_out_0_lsw()),
            16..=31 => (
                self.raw_offset() - 16,
                self.regs.pointer_to_masked_out_0_msw(),
            ),
            32..=47 => (
                self.raw_offset() - 32,
                self.regs.pointer_to_masked_out_1_lsw(),
            ),
            48..=53 => (
                self.raw_offset() - 48,
                self.regs.pointer_to_masked_out_1_msw(),
            ),
            _ => panic!("invalid MIO pin offset"),
        }
    }
}

/// Type-erased MIO pin.
#[derive(Debug)]
pub struct AnyPin {
    offset: u8,
}

impl AnyPin {
    pub(crate) const fn new(offset: u8) -> Self {
        Self { offset }
    }

    pub(crate) fn offset(&self) -> u8 {
        self.offset
    }
}

impl_peripheral!(AnyPin);

pub(crate) trait SealedPin {
    fn offset(&self) -> u8;
}

pub(crate) mod sealed {
    pub trait InputPin {}
}

/// MIO pin token accepted by the Embassy GPIO drivers.
#[allow(private_bounds)]
pub trait Pin: PeripheralType + Into<AnyPin> + SealedPin + Sized + 'static {
    #[inline]
    fn offset(&self) -> u8 {
        SealedPin::offset(self)
    }
}

/// MIO pin token that can be configured as an input.
pub trait InputPin: Pin + sealed::InputPin {}

impl SealedPin for AnyPin {
    fn offset(&self) -> u8 {
        self.offset
    }
}

impl Pin for AnyPin {}

fn enable_gpio_clock() {
    unsafe {
        slcr::with_unlocked(|slcr| {
            slcr.clk_ctrl().modify_aper_clk_ctrl(|mut val| {
                val.set_gpio_1x_clk_act(true);
                val
            });
        });
    }
}

/// Dynamic GPIO pin driver.
pub struct Flex<'d> {
    _pin: Peri<'d, AnyPin>,
    ll: LowLevelGpio,
    mode: PinMode,
}

impl<'d> Flex<'d> {
    /// Create a new flex pin from a token pin.
    pub fn new(pin: Peri<'d, impl Pin>) -> Self {
        enable_gpio_clock();
        let pin = pin.into();
        let ll = LowLevelGpio::new(PinOffset::Mio(pin.offset() as usize));
        Self {
            _pin: pin,
            ll,
            mode: PinMode::Disconnected,
        }
    }

    pub fn set_as_disconnected(&mut self) {
        self.mode = PinMode::Disconnected;
        self.ll.configure_as_disconnected();
    }

    pub fn set_as_input(&mut self) -> Result<(), PinIsOutputOnly> {
        self.configure_as_input_floating()
    }

    pub fn set_as_input_output(&mut self, level: PinState, with_internal_pullup: bool) {
        self.configure_as_output_open_drain(level, with_internal_pullup);
    }

    pub fn set_as_output(&mut self, level: PinState) {
        self.configure_as_output_push_pull(level);
    }

    pub fn configure_as_input_floating(&mut self) -> Result<(), PinIsOutputOnly> {
        commit_input_mode(&mut self.mode, PinMode::InputFloating, || {
            self.ll.configure_as_input_floating()
        })
    }

    pub fn configure_as_input_with_pull_up(&mut self) -> Result<(), PinIsOutputOnly> {
        commit_input_mode(&mut self.mode, PinMode::InputPullUp, || {
            self.ll.configure_as_input_with_pull_up()
        })
    }

    pub fn configure_as_output_push_pull(&mut self, level: PinState) {
        self.mode = PinMode::OutputPushPull;
        self.ll.configure_as_output_push_pull(level);
    }

    pub fn configure_as_output_open_drain(&mut self, level: PinState, with_internal_pullup: bool) {
        self.mode = PinMode::OutputOpenDrain;
        self.ll
            .configure_as_output_open_drain(level, with_internal_pullup);
    }

    pub fn set_high(&mut self) {
        if self.mode == PinMode::OutputOpenDrain {
            self.ll.set_high();
            self.ll.disable_output_driver();
        } else {
            self.ll.set_high();
        }
    }

    pub fn set_low(&mut self) {
        self.ll.set_low();
        if self.mode == PinMode::OutputOpenDrain {
            self.ll.enable_output_driver();
        }
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        self.ll.is_high()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self.ll.is_low()
    }

    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.ll.is_set_low()
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.ll.is_set_high()
    }

    #[inline]
    pub fn toggle(&mut self) {
        if self.is_set_high() {
            self.set_low();
        } else {
            self.set_high();
        }
    }
}

fn commit_input_mode(
    mode: &mut PinMode,
    next: PinMode,
    configure: impl FnOnce() -> Result<(), PinIsOutputOnly>,
) -> Result<(), PinIsOutputOnly> {
    configure()?;
    *mode = next;
    Ok(())
}

/// GPIO output driver.
pub struct Output<'d> {
    _pin: Peri<'d, AnyPin>,
    ll: LowLevelGpio,
}

impl<'d> Output<'d> {
    pub fn new(pin: Peri<'d, impl Pin>, initial_output: PinState) -> Self {
        enable_gpio_clock();
        let pin = pin.into();
        let mut ll = LowLevelGpio::new(PinOffset::Mio(pin.offset() as usize));
        ll.configure_as_output_push_pull(initial_output);
        Self { _pin: pin, ll }
    }

    #[inline]
    pub fn set_low(&mut self) {
        self.ll.set_low();
    }

    #[inline]
    pub fn set_high(&mut self) {
        self.ll.set_high();
    }

    #[inline]
    pub fn toggle(&mut self) {
        if self.is_set_high() {
            self.set_low();
        } else {
            self.set_high();
        }
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.ll.is_set_high()
    }

    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.ll.is_set_low()
    }
}

/// GPIO input driver.
pub struct Input<'d> {
    _pin: Peri<'d, AnyPin>,
    ll: LowLevelGpio,
}

impl<'d> Input<'d> {
    pub fn new(pin: Peri<'d, impl InputPin>) -> Result<Self, PinIsOutputOnly> {
        enable_gpio_clock();
        let pin: Peri<'d, AnyPin> = pin.into();
        let mut ll = LowLevelGpio::new(PinOffset::Mio(pin.offset() as usize));
        ll.configure_as_input_floating()?;
        Ok(Self { _pin: pin, ll })
    }

    pub fn is_high(&self) -> bool {
        self.ll.is_high()
    }

    pub fn is_low(&self) -> bool {
        self.ll.is_low()
    }
}

impl<'d> embedded_hal::digital::ErrorType for Flex<'d> {
    type Error = Infallible;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn failed_input_mode_change_preserves_previous_mode() {
        let mut mode = PinMode::OutputPushPull;

        let result = commit_input_mode(&mut mode, PinMode::InputFloating, || Err(PinIsOutputOnly));

        assert_eq!(result, Err(PinIsOutputOnly));
        assert_eq!(mode, PinMode::OutputPushPull);
    }

    #[test]
    fn successful_input_mode_change_updates_mode() {
        let mut mode = PinMode::Disconnected;

        commit_input_mode(&mut mode, PinMode::InputPullUp, || Ok(())).unwrap();

        assert_eq!(mode, PinMode::InputPullUp);
    }
}

impl<'d> embedded_hal::digital::InputPin for Flex<'d> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_low())
    }
}

impl<'d> embedded_hal::digital::OutputPin for Flex<'d> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }
}

impl<'d> embedded_hal::digital::StatefulOutputPin for Flex<'d> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_set_low())
    }
}

impl<'d> embedded_hal::digital::ErrorType for Output<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal::digital::OutputPin for Output<'d> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.ll.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.ll.set_high();
        Ok(())
    }
}

impl<'d> embedded_hal::digital::StatefulOutputPin for Output<'d> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_set_low())
    }
}

impl<'d> embedded_hal::digital::ErrorType for Input<'d> {
    type Error = Infallible;
}

impl<'d> embedded_hal::digital::InputPin for Input<'d> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_low())
    }
}

impl Flex<'static> {
    /// Persist the pin's configuration for the rest of the program's lifetime.
    pub fn persist(self) {
        core::mem::forget(self);
    }
}

impl Output<'static> {
    /// Persist the pin's configuration for the rest of the program's lifetime.
    pub fn persist(self) {
        core::mem::forget(self);
    }
}

impl Input<'static> {
    /// Persist the pin's configuration for the rest of the program's lifetime.
    pub fn persist(self) {
        core::mem::forget(self);
    }
}

impl<'d> Drop for Flex<'d> {
    fn drop(&mut self) {
        self.set_as_disconnected();
    }
}

impl<'d> Drop for Output<'d> {
    fn drop(&mut self) {
        self.ll.configure_as_disconnected();
    }
}

impl<'d> Drop for Input<'d> {
    fn drop(&mut self) {
        self.ll.configure_as_disconnected();
    }
}
