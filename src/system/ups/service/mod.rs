/// Service framework for UPS system services.
///
/// This module defines the common interface for all UPS services.

/// Common trait for UPS services that need periodic updates.
pub trait Service {
    /// Update the service state.
    ///
    /// This method should be called periodically to allow the service
    /// to perform its logic, such as monitoring sensors and controlling actuators.
    fn update(&mut self);
}

pub mod battery_monitoring;
pub mod energy_metering;
pub mod temperature_monitoring;
pub use battery_monitoring::*;
pub use energy_metering::*;
pub use temperature_monitoring::*;
