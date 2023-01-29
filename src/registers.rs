#[allow(missing_docs)]
// defined according to datasheet page 11
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
#[allow(non_camel_case_types)]
pub enum CY8C95XX_Reg {
    INPUT_PORT_0 = 0x00, // default none
    INPUT_PORT_1,
    INPUT_PORT_2,
    INPUT_PORT_3,
    INPUT_PORT_4,
    INPUT_PORT_5,
    INPUT_PORT_6,
    INPUT_PORT_7,
    /// Output Registers
    OUTPUT_PORT_0 = 0x08, // default  FFh
    OUTPUT_PORT_1,
    OUTPUT_PORT_2,
    OUTPUT_PORT_3,
    OUTPUT_PORT_4,
    OUTPUT_PORT_5,
    OUTPUT_PORT_6,
    OUTPUT_PORT_7,
    /// interrupt status registers
    INT_STATUS_PORT_0 = 0x10, // default 00h
    INT_STATUS_PORT_1,
    INT_STATUS_PORT_2,
    INT_STATUS_PORT_3,
    INT_STATUS_PORT_4,
    INT_STATUS_PORT_5,
    INT_STATUS_PORT_6,
    INT_STATUS_PORT_7,
    // one off regs
    PORT_SELECT = 0x18,             // default 00h used to select corrisponding ports for regs 0x19 - 0x23
    INT_MASK = 0x19,                // default FFh
    PWM_SELECT_OUT_PORT = 0x1A,     // default 00h
    INVERSION = 0x1B,               // default 00h
    PORT_DIR_IO = 0x1c,             // default 00h output
    DRIVE_MODE_PULL_UP = 0x1D,      // default FFh
    DRIVE_MODE_PULL_DOWN = 0x1E,    // default 00h
    DRIVE_MODE_OPEN_DRAIN_H = 0x1F, // default 00h
    DRIVE_MODE_OPEN_DRAIN_L = 0x20, // default 00h
    DRIVE_MODE_STRONG = 0x21,       // default 00h
    DRIVE_MODE_SLOW_STRONG = 0x22,  // default 00h
    DRIVE_MODE_HIGH_Z = 0x23,       // default 00h
    // RESERVED 0x24
    // RESERVED 0x25
    // RESERVED 0x26
    // RESERVED 0x27
    PWM_SELECT = 0x28,      // default 00h
    PWM_CONFIG = 0x29,      // default 00h
    PWM_PERIOD = 0x2A,      // default FFh
    PWM_PULSE_WIDTH = 0x2B, // default 80h
    PROG_DIVIDER = 0x2C,    // default FFh
    ENABLE_WDE = 0x2D,      // default 00h
    DEVICE_INFO = 0x2E,     // default 20h/40h/60h
    WATCHDOG = 0x2F,        // default 00h
    COMMAND = 0x30,         // default 00h
}

impl CY8C95XX_Reg {
}

// no special into for this enum, registers are sequentially ordered and can reference an index 1:1 safely
impl Into<usize> for CY8C95XX_Reg {
    fn into(self) -> usize {
        return self as usize;
    }
}
