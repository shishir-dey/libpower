use crate::system::ups::infra::event::Event;
use crate::system::ups::service::{BatteryEvent, Service};

/// States for the UPS system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpsState {
    /// System is off.
    Off,
    /// System is in mains mode (normal operation from mains).
    MainsMode,
    /// System is in UPS mode (battery backup).
    UpsMode,
}

/// Top-level state machine for the UPS system.
pub struct StateMachine<'a, S: Service, E: Event<Id = BatteryEvent, Payload = ()>> {
    state: UpsState,
    services: &'a mut [S],
    event_system: &'a mut E,
}

impl<'a, S: Service, E: Event<Id = BatteryEvent, Payload = ()>> StateMachine<'a, S, E> {
    /// Create a new state machine with initial state Off, given services, and event system.
    pub fn new(services: &'a mut [S], event_system: &'a mut E) -> Self {
        Self {
            state: UpsState::Off,
            services,
            event_system,
        }
    }

    /// Get the current state.
    pub fn current_state(&self) -> UpsState {
        self.state
    }

    /// Set the state.
    pub fn set_state(&mut self, state: UpsState) {
        self.state = state;
    }

    /// Update the state machine, handle events, and run services if in active state.
    pub fn update(&mut self) {
        // Handle events
        if self
            .event_system
            .is_pending(BatteryEvent::LowBatteryShutdown)
        {
            self.event_system.take(BatteryEvent::LowBatteryShutdown);
            self.state = UpsState::Off;
        }
        // Other events can be handled here

        match self.state {
            UpsState::Off => {
                // Do nothing
            }
            UpsState::MainsMode | UpsState::UpsMode => {
                // Run all services
                for service in self.services.iter_mut() {
                    service.update();
                }
            }
        }
    }
}
