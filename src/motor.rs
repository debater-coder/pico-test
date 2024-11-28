use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

pub struct Motor<S: SetDutyCycle, P: OutputPin> {
    pub pwm: S,
    pub dir: P,
}

impl<S: SetDutyCycle, P: OutputPin> Motor<S, P> {
    pub fn stop(&mut self) {
        self.pwm.set_duty_cycle_fully_off().unwrap();
    }

    pub fn speed(&mut self, speed: i8) {
        assert!(speed.abs() <= 100);

        if speed > 0 {
            self.dir.set_high().unwrap();
        } else {
            self.dir.set_low().unwrap();
        }

        self.pwm
            .set_duty_cycle_percent(speed.abs().try_into().unwrap())
            .unwrap();
    }
}
