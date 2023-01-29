//! # i2c Example
//!
//! Boilerplate i2c taken from rp-hal i2c example with added driver as part of the example
//!
//! This application demonstrates how to talk to I²C CY8C95XX devices with an RP2040 and read a single gpio port.
//!
//! example works on any rp2040 device which exposes i2c1  gpio8 and gpio9 / (physical) pin11 and pin12
//!
//! Note that in its un-configured state that the chip is in quasi-bidirectional mode, meaning pins are high and the un-grounded state of
//! the pins will reflect as a 1 in the input register which reflects the logic level on the pin.
//!
//! For this example we are reading the first 3 pins (io 0, 1, 2) of the gpio bank 0 and blink the pico led for each `1` we read.
//! if you ground physically any of these pins then you will read a 0 and the led will not blink.
//!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]
use embedded_hal::digital::v2::OutputPin;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;
// use Cy8c95xx;
// Some traits we need
use fugit::RateExtU32;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::{pac, Clock};

// our device driver
use CY8C95XX_driver::device::Cy8c95xx;
use CY8C95XX_driver::registers::CY8C95XX_Reg;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then performs a single I²C
/// write to a fixed address.
#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C
    let sda_pin = pins.gpio8.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio9.into_mode::<hal::gpio::FunctionI2C>();
    // let not_an_scl_pin = pins.gpio20.into_mode::<hal::gpio::FunctionI2C>();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        100.kHz(), //try reducing the clock if things don't seem to work right off the bat
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // in the case of this example A0 is pulled strong high, setting our soft address to 1
    let mut device = Cy8c95xx::new(i2c, 1);

    let mut pico_led = pins.gpio25.into_push_pull_output();
    loop {
        device.read_io().unwrap();
        let port = CY8C95XX_Reg::INPUT_PORT_0 as usize;
        if device.regs[port] & 1 != 0 {
            pico_led.set_high().unwrap();
        }
        delay.delay_ms(500);
        pico_led.set_low().unwrap();
        delay.delay_ms(500);
        if device.regs[port] & 2 != 0 {
            pico_led.set_high().unwrap();
        }
        delay.delay_ms(500);
        pico_led.set_low().unwrap();
        delay.delay_ms(500);

        if device.regs[port] & 4 != 0 {
            pico_led.set_high().unwrap();
        }
        delay.delay_ms(500);
        pico_led.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
