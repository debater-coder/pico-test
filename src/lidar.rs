/// # Troubleshooting
/// - Check motor direction
use rp_pico::hal::uart::{Enabled, UartDevice, UartPeripheral, ValidUartPinout};

use lib::Motor;

const SCAN_RPM: u16 = 300;
const INVALID_DATA_FLAG: u8 = 1 << 7;
const STRENGTH_WARNING_FLAG: u8 = 1 << 6;
const COMMAND_BYTE: u8 = 0xfa;

#[derive(Debug, Clone)]
pub enum LidarError {
    InvalidPacket,
    InvalidChecksum,
    InvalidData,
    InvalidAngle,
}

#[derive(Debug, Clone, Copy)]
pub struct LidarMessage {
    pub angle: u16,
    pub distance: u16,
    pub speed: u16,
    pub strength_warning: bool,
}

pub struct Lidar<'a, U: UartDevice, P: ValidUartPinout<U>> {
    motor: Motor<'a>,
    uart: UartPeripheral<Enabled, U, P>,
    previous_speed: i32,
    integral: i32,
}

impl<'a, U: UartDevice, P: ValidUartPinout<U>> Lidar<'a, U, P> {
    pub fn new(motor: Motor<'a>, uart: UartPeripheral<Enabled, U, P>) -> Self {
        Lidar {
            motor,
            uart,
            previous_speed: 0,
            integral: 0,
        }
    }

    pub fn read(&mut self) -> Option<Result<[LidarMessage; 4], LidarError>> {
        if !self.uart.uart_is_readable() {
            return None;
        }

        let mut packet = [0; 22];
        let mut index = 0;

        let mut byte = [0; 1];

        while {
            if let Err(_) = self.uart.read_full_blocking(byte.as_mut_slice()) {
                byte[0] = 0;
            };
            byte[0] == 0xfa
        } {}

        packet[index] = 0xfa;
        index += 1;

        while index < 22 {
            if let Err(_) = self.uart.read_full_blocking(byte.as_mut_slice()) {
                return Some(Err(LidarError::InvalidPacket));
            };

            if byte[0] == 0xfa {
                return Some(Err(LidarError::InvalidPacket));
            }

            packet[index] = byte[0];
            index += 1;
        }

        Some(match Self::decode_packet(&packet) {
            Err(e) => Err(e),
            Ok((msg, speed)) => {
                self.set_motor_speed(speed, 300, 2.0, 0.3, 0.3);

                Ok(msg)
            }
        })
    }

    /// result is tuple of (msgs, speed)
    fn decode_packet(packet: &[u8; 22]) -> Result<([LidarMessage; 4], i32), LidarError> {
        if !Self::check_packet_valid(packet) {
            return Err(LidarError::InvalidChecksum);
        }
        assert_eq!(packet[0], 0xfa);

        if packet[1] < 0xa0 || packet[1] > 0xf9 {
            return Err(LidarError::InvalidAngle);
        }
        let angle: u16 = (packet[1] - 0xa0) as u16 * 4;

        let angle = angle % 360;

        let speed: u16 = (packet[3] as u16) << 8 | packet[2] as u16;
        let speed = speed / 64;

        let mut lidar_msgs: [LidarMessage; 4] = [LidarMessage {
            angle: 0,
            distance: 0,
            speed: 0,
            strength_warning: true,
        }; 4]; // This isn't ideal but it works

        for (i, data_start) in [4, 8, 12, 16].iter().enumerate() {
            if packet[*data_start + 1] & INVALID_DATA_FLAG != 0 {
                return Err(LidarError::InvalidData);
            }
            lidar_msgs[i] = LidarMessage {
                angle: angle + i as u16,
                distance: ((packet[*data_start + 1] as u16 & 0x3f) << 8)
                    | packet[*data_start] as u16,
                speed,
                strength_warning: packet[*data_start + 1] & STRENGTH_WARNING_FLAG == 0,
            }
        }

        Ok((lidar_msgs, speed.try_into().unwrap()))
    }

    fn check_packet_valid(packet: &[u8; 22]) -> bool {
        let mut checksum: u32 = 0;

        for i in (0..20).skip(2) {
            checksum = (checksum << 1) + (packet[i] as u16 + (packet[i + 1] as u16 >> 8)) as u32;
        }

        checksum = (checksum & 0x07fff) + (checksum >> 15);

        return (checksum & 0xff) as u8 == packet[20] && (checksum >> 8) as u8 == packet[21];
    }

    fn set_motor_speed(&mut self, actual: i32, target: i32, kp: f32, ki: f32, kd: f32) {
        let proportional = target - actual;
        let derivative = actual - self.previous_speed;

        self.integral += proportional;
        let effort =
            (kp * proportional as f32 + kd * derivative as f32 + ki * self.integral as f32)
                + actual as f32;

        let mut effort = effort as i32;

        if effort > 100 {
            // When the motor is spinning up, integral can accumulate very fast
            self.integral = self.integral.checked_sub(proportional).unwrap_or(0); // This undoes the integral increase
            effort =
                ((kp * proportional as f32 + kd * derivative as f32 + ki * self.integral as f32)
                    + actual as f32) as i32;
        }

        self.previous_speed = actual;
        self.motor.speed(effort.min(0).max(100) as i8);
    }
}
