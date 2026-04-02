use zynq7000::gtc::MmioRegisters;

use crate::clocks::ArmClocks;

pub struct GlobalTimerCounter {
    regs: MmioRegisters<'static>,
}

unsafe impl Send for GlobalTimerCounter {}

impl GlobalTimerCounter {
    pub const unsafe fn steal_fixed(_cpu_3x2x_clk: Option<crate::Hertz>) -> Self {
        Self {
            regs: unsafe { zynq7000::gtc::Registers::new_mmio_fixed() },
        }
    }

    pub fn new(clocks: &ArmClocks) -> Self {
        unsafe { Self::steal_fixed(Some(clocks.cpu_3x2x_clk())) }
    }

    pub fn read_timer(&self) -> u64 {
        loop {
            let upper = self.regs.read_count_upper();
            let lower = self.regs.read_count_lower();
            if self.regs.read_count_upper() == upper {
                return ((upper as u64) << 32) | (lower as u64);
            }
        }
    }

    pub fn set_comparator(&mut self, comparator: u64) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_comparator_enable(false);
            ctrl
        });
        self.regs.write_comparator_upper((comparator >> 32) as u32);
        self.regs.write_comparator_lower(comparator as u32);
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_comparator_enable(true);
            ctrl
        });
    }

    pub fn enable(&mut self) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_enable(true);
            ctrl
        });
    }

    pub fn set_prescaler(&mut self, prescaler: u8) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_prescaler(prescaler);
            ctrl
        });
    }

    pub fn enable_interrupt(&mut self) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_irq_enable(true);
            ctrl
        });
    }

    pub fn disable_interrupt(&mut self) {
        self.regs.modify_ctrl(|mut ctrl| {
            ctrl.set_irq_enable(false);
            ctrl
        });
    }

    pub fn clear_interrupt_event(&mut self) {
        self.regs
            .write_isr(zynq7000::gtc::InterruptStatus::new_with_raw_value(1));
    }
}
