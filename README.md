# CY8C95XXA Driver

This driver supports the CY8C95XXA IO Expander chips from Cypress Perform, tested on the rp2040.

A simple I2C(IIC) driver that enables reads and writes to the registers for configuring the device.

some operational details on the chip:

- standard mode i2c (100kHz), stretching clock required by i2c master
- soft addressing, higher bit address pins sit on gpio, reducing gpio count
  - strong pull up = 330R or less. Weak pull up:75k-200k ohm
- factory usermode:
  - quazi-bidirectional IO
  - interrupts are not active
  - WD pin not enabled
- XRES has an internal pull-down resistor
- 8 bit registers
- read/write operations are auto incrementing to the next register - easy sequential reads/writes
- port select register for writing to port specific registers
- setting a drive mode register will override the prior setting -"last register priority" - no need to unset drive mode registers

## I2C Details

This driver supports the extendable soft addressing (page 4, page 9) these chips are designed for and the api provides the 2 base I2C addresses that are mapped to the multi-port device (MP_D) and EEPROM device (EEPROM_D) Without supporting the upper addresses in this format (Untested; not in my scope. Please feel free to add support).

```
//          (0b010000 << 1)
MP_D_ADDR     = (0x10 << 1) + (soft_address << 1) + Read/Write bit
//          (0b101000 << 1)
EEPROM_D_ADDR = (0x28 << 1) + (soft_address << 1) + Read/Write bit

```

```
|multi-port device |  EEPROM device   |
|0|1|0|0|0|0|A0|R/W|0|1|0|0|0|0|A0|R/W|
...
```

## Registers

page 11

```rust
//! Input registers
pub enum CY8C95XX_Reg {
  INPUT_PORT_0 = 0x00,                     // default none
  INPUT_PORT_1,
  INPUT_PORT_2,
  INPUT_PORT_3,
  INPUT_PORT_4,
  INPUT_PORT_5,
  INPUT_PORT_6,
  INPUT_PORT_7,
//! Output Registers
  OUTPUT_PORT_0 = 0x08,                    // default  FFh
  OUTPUT_PORT_1,
  OUTPUT_PORT_2,
  OUTPUT_PORT_3,
  OUTPUT_PORT_4,
  OUTPUT_PORT_5,
  OUTPUT_PORT_6,
  OUTPUT_PORT_7,
//! interrupt status registers
  INT_STATUS_PORT_0 = 0x10,                // default 00h
  INT_STATUS_PORT_1,
  INT_STATUS_PORT_2,
  INT_STATUS_PORT_3,
  INT_STATUS_PORT_4,
  INT_STATUS_PORT_5,
  INT_STATUS_PORT_6,
  INT_STATUS_PORT_7,
  // one off regs
  PORT_SELECT = 0x18,                      // default 00h
  INT_MASK  = 0x19,                        // default FFh
  PWM_SELECT_OUT_PORT = 0x1A,              // default 00h
  INVERSION = 0x1B,                        // default 00h
  PIN_DIR_IO = 0x1c,                       // default 00h
  DRIVE_MODE_PULL_UP = 0x1D,               // default FFh
  DRIVE_MODE_PULL_DOWN = 0x1E,             // default 00h
  DRIVE_MODE_OPEN_DRAIN_H = 0x1F,          // default 00h
  DRIVE_MODE_OPEN_DRAIN_L = 0x20,          // default 00h
  DRIVE_MODE_STRONG = 0x21,                // default 00h
  DRIVE_MODE_SLOW_STRONG = 0x22,           // default 00h
  DRIVE_MODE_HIGH_Z = 0x23,                // default 00h
// RESERVED 0x24
// RESERVED 0x25
// RESERVED 0x26
// RESERVED 0x27
  PWM_SELECT = 0x28,                       // default 00h
  PWM_CONFIG = 0x29,                       // default 00h
  PWM_PERIOD = 0x2A,                       // default FFh
  PWM_PULSE_WIDTH = 0x2B,                  // default 80h
  PROG_DIVIDER = 0x2C,                     // default FFh
  ENABLE_WDE = 0x2D,                       // default 00h
  DEVICE_INFO = 0x2E,                      // default 20h/40h/60h
  WATCHDOG = 0x2F,                         // default 00h
  COMMAND = 0x30,                          // default 00h
}
```
