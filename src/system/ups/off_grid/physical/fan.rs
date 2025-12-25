/// Fan control trait and implementations for UPS physical components.
///
/// This module provides a trait for fan control with on/off and speed management,
/// along with a default mock implementation and a microcontroller-specific implementation.

/// Trait for fan control operations.
pub trait Fan {
    /// Turn the fan on.
    fn on(&mut self);

    /// Turn the fan off.
    fn off(&mut self);

    /// Set the fan speed (0.0 to 1.0).
    fn set_speed(&mut self, speed: f32);

    /// Get the current fan speed.
    fn get_speed(&self) -> f32;

    /// Check if the fan is on.
    fn is_on(&self) -> bool;
}

/// Mock implementation of the Fan trait for testing and default behavior.
#[derive(Debug, Clone, Default)]
pub struct FanMock {
    is_on: bool,
    speed: f32,
}

impl Fan for FanMock {
    fn on(&mut self) {
        self.is_on = true;
    }

    fn off(&mut self) {
        self.is_on = false;
        self.speed = 0.0;
    }

    fn set_speed(&mut self, speed: f32) {
        if self.is_on {
            self.speed = speed.clamp(0.0, 1.0);
        }
    }

    fn get_speed(&self) -> f32 {
        self.speed
    }

    fn is_on(&self) -> bool {
        self.is_on
    }
}
