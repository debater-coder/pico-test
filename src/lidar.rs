use rp_pico::hal::uart::{Enabled, UartDevice, UartPeripheral, ValidUartPinout};

use crate::motor::Motor;

pub struct Lidar<'a, U: UartDevice, P: ValidUartPinout<U>> {
    motor: Motor<'a>,
    uart: UartPeripheral<Enabled, U, P>,
}

impl<'a, U: UartDevice, P: ValidUartPinout<U>> Lidar<'a, U, P> {
    // Implementation details go here
}
