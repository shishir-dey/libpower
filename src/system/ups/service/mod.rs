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

pub mod energy_metering;
pub mod overtemperature_protection;
pub use energy_metering::*;
pub use overtemperature_protection::*;
