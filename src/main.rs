//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod lidar;
mod motor;

use bsp::entry;
use bsp::hal;
use defmt::*;
use defmt_rtt as _;
use motor::Motor;
use panic_probe as _;

use rp_pico::hal::uart::DataBits;
use rp_pico::hal::uart::StopBits;
use rp_pico::hal::uart::UartConfig;
// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    fugit::RateExtU32,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let lidar_pwm = pins.gpio0;
    let lidar_dir = pins.gpio1;

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio4.into_function(), // dummy
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio5.into_function(),
    );
    let mut lidar_uart: hal::uart::UartPeripheral<
        hal::uart::Enabled,
        pac::UART1,
        (
            hal::gpio::Pin<hal::gpio::bank0::Gpio4, hal::gpio::FunctionUart, hal::gpio::PullDown>,
            hal::gpio::Pin<hal::gpio::bank0::Gpio5, hal::gpio::FunctionUart, hal::gpio::PullDown>,
        ),
    > = hal::uart::UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let lidar_pwm_slice = &mut pwm_slices.pwm0;

    lidar_pwm_slice.set_ph_correct();
    lidar_pwm_slice.enable();

    let channel = &mut lidar_pwm_slice.channel_a;
    channel.output_to(lidar_pwm);

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.

    let mut motor = Motor {
        pwm: channel,
        dir: &mut lidar_dir.into_push_pull_output(),
    };

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Hamzah Ahmed")
            .product("LIDAR driver")
            .serial_number("Pico-1")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    loop {
        if lidar_uart.uart_is_readable() {
            let buffer = &mut [0_u8; 256];
            if let Ok(bytes_read) = lidar_uart.read_raw(buffer.as_mut_slice()) {
                let _ = serial.write(&buffer[..bytes_read]);
            }
        }
        usb_dev.poll(&mut [&mut serial]);
    }
}
