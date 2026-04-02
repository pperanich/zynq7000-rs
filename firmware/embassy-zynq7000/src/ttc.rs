use core::{convert::Infallible, marker::PhantomData};

use arbitrary_int::{prelude::*, u4};
use embassy_hal_internal::{Peri, PeripheralType};
use embedded_hal::pwm::SetDutyCycle as _;

use crate::{Hertz, clocks::Clocks, pac};

#[doc(hidden)]
pub trait SealedInstance {
    fn regs() -> pac::ttc::MmioRegisters<'static>;
}

/// TTC peripheral instance token.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {}

/// Native TTC wrapper built from an Embassy peripheral token.
pub struct Ttc<'d, T: Instance> {
    pub ch0: Channel<'d, T, 0>,
    pub ch1: Channel<'d, T, 1>,
    pub ch2: Channel<'d, T, 2>,
}

/// Native TTC channel wrapper.
pub struct Channel<'d, T: Instance, const N: usize> {
    regs: pac::ttc::MmioRegisters<'static>,
    id: usize,
    _phantom: PhantomData<&'d mut T>,
}

/// Native PWM wrapper.
pub struct Pwm<'d, T: Instance, const N: usize> {
    channel: Channel<'d, T, N>,
    ref_clk: Hertz,
    _phantom: PhantomData<&'d mut T>,
}

#[derive(Debug, thiserror::Error)]
pub enum FrequencyError {
    #[error("frequency is zero")]
    Zero,
    #[error("frequency is out of range for TTC")]
    OutOfRange,
}

#[derive(Debug, thiserror::Error, Clone, Copy, PartialEq, Eq)]
pub enum DutyCycleError {
    #[error("duty cycle percent must be in the range 0..=100")]
    InvalidPercent,
}

impl<'d, T: Instance> Ttc<'d, T> {
    pub fn new(_ttc: Peri<'d, T>) -> Self {
        let regs = T::regs();
        Self {
            ch0: Channel {
                regs: unsafe { regs.clone() },
                id: 0,
                _phantom: PhantomData,
            },
            ch1: Channel {
                regs: unsafe { regs.clone() },
                id: 1,
                _phantom: PhantomData,
            },
            ch2: Channel {
                regs,
                id: 2,
                _phantom: PhantomData,
            },
        }
    }
}

impl<'d, T: Instance, const N: usize> Channel<'d, T, N> {
    pub fn read_counter(&self) -> u16 {
        self.regs.read_current_counter(self.id).unwrap().count()
    }
}

impl<'d, T: Instance, const N: usize> Pwm<'d, T, N> {
    pub fn new_with_clocks(
        channel: Channel<'d, T, N>,
        clocks: &Clocks,
        freq: Hertz,
    ) -> Result<Self, FrequencyError> {
        Self::new_generic(channel, clocks.cpu_1x_clk(), freq)
    }

    pub fn new_generic(
        channel: Channel<'d, T, N>,
        ref_clk: Hertz,
        freq: Hertz,
    ) -> Result<Self, FrequencyError> {
        if freq.raw() == 0 {
            return Err(FrequencyError::Zero);
        }
        let id = channel.id;
        let mut pwm = Self {
            channel,
            ref_clk,
            _phantom: PhantomData,
        };
        let (prescaler_reg, tick_val) = calc_prescaler_reg_and_interval_ticks(ref_clk, freq)?;
        pwm.set_up_and_configure_pwm(id, prescaler_reg, tick_val);
        Ok(pwm)
    }

    pub fn set_frequency(&mut self, freq: Hertz) -> Result<(), FrequencyError> {
        if freq.raw() == 0 {
            return Err(FrequencyError::Zero);
        }
        let (prescaler_reg, tick_val) = calc_prescaler_reg_and_interval_ticks(self.ref_clk, freq)?;
        self.set_up_and_configure_pwm(self.channel.id, prescaler_reg, tick_val);
        Ok(())
    }

    pub fn set_duty_cycle_percent(&mut self, percent: u8) -> Result<(), DutyCycleError> {
        if percent > 100 {
            return Err(DutyCycleError::InvalidPercent);
        }
        let duty = ((self.max_duty_cycle() as u32) * (percent as u32) / 100) as u16;
        self.set_duty_cycle(duty);
        Ok(())
    }

    pub fn max_duty_cycle(&self) -> u16 {
        self.channel
            .regs
            .read_interval_value(self.channel.id)
            .unwrap()
            .value()
    }

    pub fn set_duty_cycle(&mut self, duty: u16) {
        self.channel
            .regs
            .modify_cnt_ctrl(self.channel.id, |mut val| {
                val.set_disable(true);
                val
            })
            .unwrap();
        self.channel
            .regs
            .write_match_value_0(
                self.channel.id,
                zynq7000::ttc::RwValue::new_with_raw_value(duty as u32),
            )
            .unwrap();
        self.channel
            .regs
            .modify_cnt_ctrl(self.channel.id, |mut val| {
                val.set_disable(false);
                val.set_reset(true);
                val
            })
            .unwrap();
    }

    fn set_up_and_configure_pwm(&mut self, id: usize, prescaler_reg: Option<u4>, tick_val: u16) {
        self.channel
            .regs
            .write_cnt_ctrl(id, zynq7000::ttc::CounterControl::new_with_raw_value(1))
            .unwrap();
        self.channel
            .regs
            .write_clk_cntr(
                id,
                zynq7000::ttc::ClockControl::builder()
                    .with_ext_clk_edge(false)
                    .with_clk_src(zynq7000::ttc::ClockSource::Pclk)
                    .with_prescaler(prescaler_reg.unwrap_or(u4::new(0)))
                    .with_prescale_enable(prescaler_reg.is_some())
                    .build(),
            )
            .unwrap();
        self.channel
            .regs
            .write_interval_value(
                id,
                zynq7000::ttc::RwValue::new_with_raw_value(tick_val as u32),
            )
            .unwrap();
        self.channel
            .regs
            .write_match_value_0(id, zynq7000::ttc::RwValue::new_with_raw_value(0))
            .unwrap();
        self.channel
            .regs
            .write_cnt_ctrl(
                id,
                zynq7000::ttc::CounterControl::builder()
                    .with_wave_polarity(zynq7000::ttc::WavePolarity::LowToHighOnMatch1)
                    .with_wave_enable_n(zynq7000::ttc::WaveEnable::Enable)
                    .with_reset(true)
                    .with_match_enable(true)
                    .with_decrementing(false)
                    .with_mode(zynq7000::ttc::Mode::Interval)
                    .with_disable(false)
                    .build(),
            )
            .unwrap();
    }
}

fn calc_prescaler_reg_and_interval_ticks(
    mut ref_clk: Hertz,
    freq: Hertz,
) -> Result<(Option<u4>, u16), FrequencyError> {
    if freq.raw() >= ref_clk.raw() {
        return Err(FrequencyError::OutOfRange);
    }
    let mut prescaler: Option<u32> = None;
    let mut tick_val = ref_clk / freq;
    while tick_val > u16::MAX as u32 {
        ref_clk /= 2;
        match prescaler {
            Some(val) => {
                if val == u4::MAX.as_u32() {
                    return Err(FrequencyError::OutOfRange);
                }
                prescaler = Some(val + 1);
            }
            None => prescaler = Some(0),
        }
        tick_val = ref_clk / freq;
    }
    if tick_val == 0 {
        return Err(FrequencyError::OutOfRange);
    }
    Ok((prescaler.map(|v| u4::new(v as u8)), tick_val as u16))
}

impl<'d, T: Instance, const N: usize> embedded_hal::pwm::ErrorType for Pwm<'d, T, N> {
    type Error = Infallible;
}

impl<'d, T: Instance, const N: usize> embedded_hal::pwm::SetDutyCycle for Pwm<'d, T, N> {
    fn max_duty_cycle(&self) -> u16 {
        self.max_duty_cycle()
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.set_duty_cycle(duty);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prescaler_calc_rejects_zero_or_too_fast_frequencies() {
        assert!(matches!(
            calc_prescaler_reg_and_interval_ticks(Hertz::Hz(1_000_000), Hertz::Hz(1_000_000)),
            Err(FrequencyError::OutOfRange)
        ));
        assert!(matches!(
            calc_prescaler_reg_and_interval_ticks(Hertz::Hz(1_000_000), Hertz::Hz(2_000_000)),
            Err(FrequencyError::OutOfRange)
        ));
    }

    #[test]
    fn prescaler_calc_uses_no_prescaler_when_interval_fits() {
        let (prescaler, ticks) =
            calc_prescaler_reg_and_interval_ticks(Hertz::Hz(100_000_000), Hertz::Hz(1_000))
                .unwrap();

        assert_eq!(prescaler, None);
        assert_eq!(ticks, 100_000);
    }

    #[test]
    fn prescaler_calc_adds_prescaler_when_needed() {
        let (prescaler, ticks) =
            calc_prescaler_reg_and_interval_ticks(Hertz::Hz(100_000_000), Hertz::Hz(10)).unwrap();

        assert_eq!(prescaler, Some(u4::new(7)));
        assert_eq!(ticks, 39_062);
    }

    #[test]
    fn prescaler_calc_reports_unrepresentable_range() {
        assert!(matches!(
            calc_prescaler_reg_and_interval_ticks(Hertz::Hz(1_000_000_000), Hertz::Hz(1)),
            Err(FrequencyError::OutOfRange)
        ));
    }
}
