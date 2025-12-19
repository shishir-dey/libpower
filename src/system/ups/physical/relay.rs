/// Relay control trait and implementations for UPS physical components.
///
/// This module provides traits and structs for controlling relays 1 and 2,
/// with on/off and toggle functionality, along with mock and hardware implementations.

/// Trait for relay control operations.
pub trait Relay {
    /// Turn the relay on.
    fn on(&mut self);

    /// Turn the relay off.
    fn off(&mut self);

    /// Toggle the relay state.
    fn toggle(&mut self);

    /// Check if the relay is on.
    fn is_on(&self) -> bool;
}

/// Mock implementation for Relay 1.
#[derive(Debug, Clone, Default)]
pub struct Relay1Mock {
    state: bool,
}

impl Relay for Relay1Mock {
    fn on(&mut self) {
        self.state = true;
    }

    fn off(&mut self) {
        self.state = false;
    }

    fn toggle(&mut self) {
        self.state = !self.state;
    }

    fn is_on(&self) -> bool {
        self.state
    }
}

/// Mock implementation for Relay 2.
#[derive(Debug, Clone, Default)]
pub struct Relay2Mock {
    state: bool,
}

impl Relay for Relay2Mock {
    fn on(&mut self) {
        self.state = true;
    }

    fn off(&mut self) {
        self.state = false;
    }

    fn toggle(&mut self) {
        self.state = !self.state;
    }

    fn is_on(&self) -> bool {
        self.state
    }
}
