use crate::CY8C95XX_Reg;
use embedded_hal::blocking::i2c::WriteRead;
#[allow(non_camel_case_types)]
const MP_D_ADDR: u8 = 0x10 << 1;
const EEPROM_D_ADDR: u8 = 0x28 << 1;

// provides a utility set of functions for reading from the device
pub struct Cy8c95xx<'a, I2C> {
    i2c: &'a mut I2C,
    soft_addr: u8,
    mp_addr: u8,     // 7 bit addresses
    eeprom_addr: u8, // 7 bit addresses
    // register values ordered same as datasheet
    input_port: [u8; 8],
    output_port: [u8; 8],
    int_status: [u8; 8], //interrupt status
    int_mask: [u8; 8], //? this is used in tandem with port select register but I really don't know what that means
    inversion: [u8; 8],
    port_dir: [u8; 8],
    drive_mode: [u8; 6],
    pub regs: [u8; 0x30], // value of the last register COMMAND in here we store any results from the registers
}

impl<'a, I2C, E> Cy8c95xx<'a, I2C>
where
    I2C: WriteRead<Error = E>,
{
    pub fn new(i2c: &'a mut I2C, soft_addr: u8) -> Self {
        let mut mp_addr = MP_D_ADDR;
        if soft_addr <= 31 {
            mp_addr |= soft_addr;
        } else {
            mp_addr = soft_addr;
        }
        let mut eeprom_addr = MP_D_ADDR;
        if soft_addr <= 64 {
            eeprom_addr |= soft_addr;
        } else {
            eeprom_addr = soft_addr;
        }
        return Self {
            i2c,
            soft_addr,
            mp_addr,
            eeprom_addr,
            regs: [0; 0x30],
            input_port: [0; 8],
            output_port: [0; 8],
            int_status: [0; 8],
            int_mask: [0; 8],
            inversion: [0; 8],
            port_dir: [0; 8],
            drive_mode: [0; 6],
        };
    }

    // initialize our chip. Validate the product id with getting  Device ID/Status, set up interrupt mask, configure our gports.
    // index of the arrays corrisponds to the bit # of the port select register. 0 being the LSb
    // initalize handles the most likely initialization, you can write to the chip for other registers, eg the inversion register configuring pwm, etc.,
    pub fn initialize(&mut self, interrupt_masks: [u8; 8]) -> Result<(), E> {
        let end_reg: usize = CY8C95XX_Reg::DEVICE_INFO as usize + 1;
        self.i2c.write_read(
            self.mp_addr,
            &[CY8C95XX_Reg::DEVICE_INFO as u8],
            &mut self.regs[CY8C95XX_Reg::DEVICE_INFO.into()..end_reg],
        )
    }
    // reads all input port registers and writes the the reg property at addresses where input port register address corrispond
    pub fn read_all_regs(&mut self) -> Result<(), E> {
        self.i2c.write_read(
            self.mp_addr,
            &[CY8C95XX_Reg::INPUT_PORT_0 as u8],
            &mut self.regs
                [CY8C95XX_Reg::INPUT_PORT_0 as usize..CY8C95XX_Reg::DRIVE_MODE_HIGH_Z as usize + 1],
        );
        // skipping the reserved registers
        self.i2c.write_read(
            self.mp_addr,
            &[CY8C95XX_Reg::PWM_SELECT as u8], //first reg
            &mut self.regs[CY8C95XX_Reg::PWM_SELECT as usize..],
        )
    }
    // reads all input port registers and writes the the reg property at addresses where input port register address corrispond
    pub fn read_all_int_status(&mut self) -> Result<(), E> {
        let end_reg: usize = CY8C95XX_Reg::INT_STATUS_PORT_7 as usize + 1;
        self.i2c.write_read(
            self.mp_addr,
            &[CY8C95XX_Reg::INPUT_PORT_0 as u8], //first reg
            &mut self.regs[CY8C95XX_Reg::INT_STATUS_PORT_0 as usize..end_reg],
        )
    }
    // reads all input port registers and writes the the reg property at addresses where input port register address corrispond
    pub fn read_io(&mut self) -> Result<(), E> {
        self.i2c.write_read(
            self.mp_addr,
            &[CY8C95XX_Reg::INPUT_PORT_0 as u8],
            &mut self.regs
                [CY8C95XX_Reg::INPUT_PORT_0 as usize..CY8C95XX_Reg::INPUT_PORT_7 as usize + 1],
        )
    }

    /// basically a simple passthrough of the i2c write_read function
    pub fn write_read_mp(&mut self, write_buf: &[u8], read_buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(self.mp_addr, write_buf, read_buf)
    }
    /// basically a simple passthrough of the i2c write_read function
    pub fn write_read_eeprom(&mut self, write_buf: &[u8], read_buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(self.eeprom_addr, write_buf, read_buf)
    }
}
