use arbitrary_int::traits::Integer;
use once_cell::sync::OnceCell;
use zynq7000::slcr::{
    ClockControlRegisters,
    clocks::{
        Bypass, ClockRatioSelectReg, CpuClockRatio, DualCommonPeriphIoClockControl, PllControl,
        PllStatus, SrcSelArm, SrcSelIo,
    },
};

use crate::Hertz;

#[derive(Debug, Clone, Copy)]
pub struct ArmClocks {
    cpu_1x_clk: Hertz,
    cpu_3x2x_clk: Hertz,
}

impl ArmClocks {
    pub const fn cpu_1x_clk(&self) -> Hertz {
        self.cpu_1x_clk
    }

    pub const fn cpu_3x2x_clk(&self) -> Hertz {
        self.cpu_3x2x_clk
    }
}

#[derive(Debug, Clone, Copy)]
pub struct IoClocks {
    io_pll_clk: Hertz,
    arm_pll_clk: Hertz,
    ddr_pll_clk: Hertz,
    sdio_clk: Hertz,
    uart_clk: Hertz,
}

impl IoClocks {
    pub const fn ref_clk(&self) -> Hertz {
        self.io_pll_clk
    }

    pub const fn io_pll_clk(&self) -> Hertz {
        self.io_pll_clk
    }

    pub const fn arm_pll_clk(&self) -> Hertz {
        self.arm_pll_clk
    }

    pub const fn ddr_pll_clk(&self) -> Hertz {
        self.ddr_pll_clk
    }

    pub const fn selected_ref_clk(&self, src_sel: SrcSelIo) -> Hertz {
        match src_sel {
            SrcSelIo::IoPll | SrcSelIo::IoPllAlt => self.io_pll_clk,
            SrcSelIo::ArmPll => self.arm_pll_clk,
            SrcSelIo::DdrPll => self.ddr_pll_clk,
        }
    }

    pub const fn sdio_clk(&self) -> Hertz {
        self.sdio_clk
    }

    pub const fn uart_clk(&self) -> Hertz {
        self.uart_clk
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Clocks {
    arm_clocks: ArmClocks,
    io_clocks: IoClocks,
}

impl Clocks {
    pub const fn arm_clocks(&self) -> &ArmClocks {
        &self.arm_clocks
    }

    pub const fn io_clocks(&self) -> &IoClocks {
        &self.io_clocks
    }

    pub const fn cpu_1x_clk(&self) -> Hertz {
        self.arm_clocks.cpu_1x_clk()
    }

    pub const fn cpu_3x2x_clk(&self) -> Hertz {
        self.arm_clocks.cpu_3x2x_clk()
    }

    pub const fn uart_clk(&self) -> Hertz {
        self.io_clocks.uart_clk()
    }

    pub const fn sdio_clk(&self) -> Hertz {
        self.io_clocks.sdio_clk()
    }
}

static CLOCKS: OnceCell<Clocks> = OnceCell::new();

#[derive(Debug, Copy, Clone)]
enum ClockModuleId {
    Arm,
    Sdio,
    Uart,
}

#[derive(Debug, thiserror::Error)]
#[error("divisor is zero for {0:?}")]
pub struct DivisorZero(ClockModuleId);

#[derive(Debug, Copy, Clone)]
pub enum PllId {
    Arm,
    Io,
    Ddr,
}

#[derive(Debug, thiserror::Error)]
pub enum ClockReadError {
    #[error("detected zero PLL feedback divisor")]
    PllFeedbackZero,
    #[error("{0:?} PLL is bypassed, powered down, held in reset, or unlocked")]
    InactivePll(PllId),
    #[error(transparent)]
    DivisorZero(#[from] DivisorZero),
}

pub(crate) fn read_from_regs(ps_clk_freq: Hertz) -> Result<Clocks, ClockReadError> {
    let clk_regs = unsafe { ClockControlRegisters::new_mmio_fixed() };

    let arm_pll_cfg = clk_regs.read_arm_pll_ctrl();
    let io_pll_cfg = clk_regs.read_io_pll_ctrl();
    let ddr_pll_cfg = clk_regs.read_ddr_pll_ctrl();
    let pll_status = clk_regs.read_pll_status();

    let arm_clk_ctrl = clk_regs.read_arm_clk_ctrl();
    if arm_clk_ctrl.divisor().as_u32() == 0 {
        return Err(DivisorZero(ClockModuleId::Arm).into());
    }
    let arm_pll_clk = pll_output(ps_clk_freq, arm_pll_cfg);
    let io_pll_clk = pll_output(ps_clk_freq, io_pll_cfg);
    let ddr_pll_clk = pll_output(ps_clk_freq, ddr_pll_cfg);

    let arm_base_clk = match arm_clk_ctrl.srcsel() {
        SrcSelArm::ArmPll | SrcSelArm::ArmPllAlt => {
            validate_pll_state(PllId::Arm, arm_pll_cfg, pll_status)?;
            require_pll_output(arm_pll_clk)?
        }
        SrcSelArm::DdrPll => {
            validate_pll_state(PllId::Ddr, ddr_pll_cfg, pll_status)?;
            require_pll_output(ddr_pll_clk)?
        }
        SrcSelArm::IoPll => {
            validate_pll_state(PllId::Io, io_pll_cfg, pll_status)?;
            require_pll_output(io_pll_clk)?
        }
    };
    let arm_clk_divided = arm_base_clk / arm_clk_ctrl.divisor().as_u32();
    let clk_sel: ClockRatioSelectReg = clk_regs.read_clk_ratio_select();
    let arm_clocks = match clk_sel.sel() {
        CpuClockRatio::FourToTwoToOne => ArmClocks {
            cpu_1x_clk: arm_clk_divided / 4,
            cpu_3x2x_clk: arm_clk_divided / 2,
        },
        CpuClockRatio::SixToTwoToOne => ArmClocks {
            cpu_1x_clk: arm_clk_divided / 6,
            cpu_3x2x_clk: arm_clk_divided / 2,
        },
    };

    let handle_dual_io_clock = |clk_ctrl: DualCommonPeriphIoClockControl,
                                module_id: ClockModuleId|
     -> Result<Hertz, ClockReadError> {
        if clk_ctrl.divisor().as_u32() == 0 {
            return Err(DivisorZero(module_id).into());
        }
        match clk_ctrl.srcsel() {
            SrcSelIo::IoPll | SrcSelIo::IoPllAlt => {
                validate_pll_state(PllId::Io, io_pll_cfg, pll_status)?;
                Ok(require_pll_output(io_pll_clk)? / clk_ctrl.divisor().as_u32())
            }
            SrcSelIo::ArmPll => {
                validate_pll_state(PllId::Arm, arm_pll_cfg, pll_status)?;
                Ok(require_pll_output(arm_pll_clk)? / clk_ctrl.divisor().as_u32())
            }
            SrcSelIo::DdrPll => {
                validate_pll_state(PllId::Ddr, ddr_pll_cfg, pll_status)?;
                Ok(require_pll_output(ddr_pll_clk)? / clk_ctrl.divisor().as_u32())
            }
        }
    };

    let sdio_clk = handle_dual_io_clock(clk_regs.read_sdio_clk_ctrl(), ClockModuleId::Sdio)?;
    let uart_clk = handle_dual_io_clock(clk_regs.read_uart_clk_ctrl(), ClockModuleId::Uart)?;

    Ok(Clocks {
        arm_clocks,
        io_clocks: IoClocks {
            io_pll_clk: io_pll_clk.unwrap_or(Hertz::from_raw(0)),
            arm_pll_clk: arm_pll_clk.unwrap_or(Hertz::from_raw(0)),
            ddr_pll_clk: ddr_pll_clk.unwrap_or(Hertz::from_raw(0)),
            sdio_clk,
            uart_clk,
        },
    })
}

pub(crate) fn init(clocks: Clocks) {
    let _ = CLOCKS.set(clocks);
}

/// Return the frozen clock tree captured during [`crate::init`].
pub fn get() -> &'static Clocks {
    CLOCKS.get().expect("embassy-zynq7000::init not called")
}

fn validate_pll_state(id: PllId, pll: PllControl, status: PllStatus) -> Result<(), ClockReadError> {
    let locked = match id {
        PllId::Arm => status.arm_pll_lock(),
        PllId::Io => status.io_pll_lock(),
        PllId::Ddr => status.drr_pll_lock(),
    };

    if pll.pwrdwn() || pll.reset() || !matches!(pll.bypass(), Bypass::NotBypassed) || !locked {
        return Err(ClockReadError::InactivePll(id));
    }

    Ok(())
}

fn require_pll_output(output: Option<Hertz>) -> Result<Hertz, ClockReadError> {
    output.ok_or(ClockReadError::PllFeedbackZero)
}

fn pll_output(ps_clk_freq: Hertz, pll: PllControl) -> Option<Hertz> {
    if pll.fdiv().as_u32() == 0 {
        return None;
    }

    Some(ps_clk_freq * pll.fdiv().into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pll_output_reports_zero_feedback_divider_as_missing() {
        let pll = PllControl::new_with_raw_value(0);

        assert_eq!(pll_output(Hertz::from_raw(33_333_333), pll), None);
    }
}
