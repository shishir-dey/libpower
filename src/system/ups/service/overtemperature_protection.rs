use crate::system::ups::physical::fan::Fan;
use crate::system::ups::physical::temperature::TemperatureSensor;

use super::Service;

/// Overtemperature protection service that monitors temperature sensors and controls a fan.
///
/// The service turns the fan on when the maximum temperature exceeds the threshold,
/// and turns it off when the temperature falls below the threshold minus hysteresis.
pub struct OvertemperatureProtectionService<T1, T2, F>
where
    T1: TemperatureSensor,
    T2: TemperatureSensor,
    F: Fan,
{
    temp1: T1,
    temp2: T2,
    fan: F,
    threshold_on: f32,
    hysteresis: f32,
}

impl<T1, T2, F> OvertemperatureProtectionService<T1, T2, F>
where
    T1: TemperatureSensor,
    T2: TemperatureSensor,
    F: Fan,
{
    /// Create a new overtemperature protection service.
    ///
    /// # Arguments
    ///
    /// * `temp1` - First temperature sensor
    /// * `temp2` - Second temperature sensor
    /// * `fan` - Fan controller
    /// * `threshold_on` - Temperature threshold to turn fan on (in Celsius)
    /// * `hysteresis` - Hysteresis value for turning fan off (in Celsius)
    pub fn new(temp1: T1, temp2: T2, fan: F, threshold_on: f32, hysteresis: f32) -> Self {
        Self {
            temp1,
            temp2,
            fan,
            threshold_on,
            hysteresis,
        }
    }

    /// Update the overtemperature protection logic.
    ///
    /// Checks the maximum temperature from both sensors and controls the fan accordingly.
    pub fn update(&mut self) {
        let temp1 = self.temp1.get_temperature();
        let temp2 = self.temp2.get_temperature();
        let max_temp = temp1.max(temp2);
        let threshold_off = self.threshold_on - self.hysteresis;

        if max_temp > self.threshold_on {
            self.fan.on();
            self.fan.set_speed(1.0);
        } else if max_temp < threshold_off {
            self.fan.off();
        }
        // If between threshold_off and threshold_on, maintain current fan state
    }
}

impl<T1, T2, F> Service for OvertemperatureProtectionService<T1, T2, F>
where
    T1: TemperatureSensor,
    T2: TemperatureSensor,
    F: Fan,
{
    fn update(&mut self) {
        self.update();
    }
}
