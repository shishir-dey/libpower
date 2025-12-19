/// Buzzer control trait and implementations for UPS physical components.
///
/// This module provides a trait for buzzer control with on/off and beep modes,
/// along with a default mock implementation and a microcontroller-specific implementation.

/// Buzzer modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BuzzerMode {
    #[default]
    Off,
    On,
    FastBeep,
    SlowBeep,
}

/// Trait for buzzer control operations.
pub trait Buzzer {
    /// Turn the buzzer on continuously.
    fn on(&mut self);

    /// Turn the buzzer off.
    fn off(&mut self);

    /// Set fast beep mode.
    fn fast_beep(&mut self);

    /// Set slow beep mode.
    fn slow_beep(&mut self);

    /// Set the buzzer mode.
    fn set_mode(&mut self, mode: BuzzerMode);

    /// Get the current buzzer mode.
    fn get_mode(&self) -> BuzzerMode;

    /// Check if the buzzer is active (not off).
    fn is_active(&self) -> bool;
}

/// Mock implementation of the Buzzer trait for testing and default behavior.
#[derive(Debug, Clone, Default)]
pub struct BuzzerMock {
    mode: BuzzerMode,
}

impl Buzzer for BuzzerMock {
    fn on(&mut self) {
        self.mode = BuzzerMode::On;
    }

    fn off(&mut self) {
        self.mode = BuzzerMode::Off;
    }

    fn fast_beep(&mut self) {
        self.mode = BuzzerMode::FastBeep;
    }

    fn slow_beep(&mut self) {
        self.mode = BuzzerMode::SlowBeep;
    }

    fn set_mode(&mut self, mode: BuzzerMode) {
        self.mode = mode;
    }

    fn get_mode(&self) -> BuzzerMode {
        self.mode
    }

    fn is_active(&self) -> bool {
        self.mode != BuzzerMode::Off
    }
}
