use crate::system::ups::physical::clock::Clock;
use crate::system::ups::{PowerInterface, UPS};

/// Energy metering service for calculating energy consumption.
pub struct EnergyMetering<'a, C: Clock> {
    ups: &'a UPS<'a>,
    clock: &'a C,
}

impl<'a, C: Clock> EnergyMetering<'a, C> {
    /// Create a new energy metering service.
    pub fn new(ups: &'a UPS<'a>, clock: &'a C) -> Self {
        Self { ups, clock }
    }

    /// Get the solar energy generated in kWh.
    pub fn solar_energy_kwh(&self) -> f64 {
        if let Some(solar) = self.ups.solar {
            // Assuming energy_imported gives Wh, convert to kWh
            solar.energy_imported() / 1000.0
        } else {
            0.0
        }
    }

    /// Calculate energy consumption in kWh based on load and time.
    /// This is a simplified calculation assuming constant power over time.
    pub fn load_energy_consumption_kwh(&self) -> f64 {
        if let Some(load) = self.ups.load {
            let power_w = load.active_power().abs() as f64;
            // For simplicity, assume 1 hour consumption, but in reality, integrate over time
            // Here, just return power in kW (since kWh = kW * h, but h=1 for demo)
            power_w / 1000.0
        } else {
            0.0
        }
    }

    /// Get current timestamp in milliseconds for logging.
    pub fn current_timestamp_ms(&self) -> u64 {
        (self.clock.year() as u64 * 365 * 24 * 60 * 60 * 1000) + // Simplified
        (self.clock.month() as u64 * 30 * 24 * 60 * 60 * 1000) + // Simplified
        (self.clock.day() as u64 * 24 * 60 * 60 * 1000) +
        (self.clock.hour() as u64 * 60 * 60 * 1000) +
        (self.clock.minute() as u64 * 60 * 1000) +
        (self.clock.second() as u64 * 1000) +
        self.clock.millisecond() as u64
    }
}
