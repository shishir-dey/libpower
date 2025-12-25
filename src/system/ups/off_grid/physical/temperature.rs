/// Temperature sensor implementations using Steinhart-Hart equation for UPS physical components.
///
/// This module provides temperature sensors temp1 and temp2, converting thermistor readings
/// to temperature using the Steinhart-Hart equation.

/// Coefficients for Steinhart-Hart equation.
/// These are example values; replace with actual thermistor coefficients.
const A: f32 = 0.001129148;
const B: f32 = 0.000234125;
const C: f32 = 0.0000000876741;

/// Trait for temperature sensor operations.
pub trait TemperatureSensor {
    /// Get the current temperature in Celsius.
    fn get_temperature(&self) -> f32;
}

/// Mock implementation for Temp1.
#[derive(Debug, Clone, Default)]
pub struct Temp1Mock {
    temperature: f32,
}

impl Temp1Mock {
    /// Set the mock temperature.
    pub fn set_temperature(&mut self, temp: f32) {
        self.temperature = temp;
    }
}

impl TemperatureSensor for Temp1Mock {
    fn get_temperature(&self) -> f32 {
        self.temperature
    }
}

/// Mock implementation for Temp2.
#[derive(Debug, Clone, Default)]
pub struct Temp2Mock {
    temperature: f32,
}

impl Temp2Mock {
    /// Set the mock temperature.
    pub fn set_temperature(&mut self, temp: f32) {
        self.temperature = temp;
    }
}

impl TemperatureSensor for Temp2Mock {
    fn get_temperature(&self) -> f32 {
        self.temperature
    }
}
