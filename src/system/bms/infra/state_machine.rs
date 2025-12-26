use super::event::BmsEvent;

/// BMS operational states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BmsState {
    /// System initializing
    Init,
    /// Idle, waiting for charger or discharge
    Idle,
    /// Precharging
    Precharge,
    /// Charging
    Charging,
    /// Discharging
    Discharging,
    /// Balancing cells
    Balancing,
    /// Fault state
    Fault,
    /// Shutdown
    Shutdown,
}

/// State machine for BMS operation.
pub struct BmsStateMachine {
    current_state: BmsState,
}

impl Default for BmsStateMachine {
    fn default() -> Self {
        Self {
            current_state: BmsState::Init,
        }
    }
}

impl BmsStateMachine {
    /// Get the current state.
    pub fn current_state(&self) -> BmsState {
        self.current_state
    }

    /// Process an event and transition to new state if applicable.
    pub fn process_event(&mut self, event: BmsEvent) {
        self.current_state = match (self.current_state, event) {
            (BmsState::Init, BmsEvent::Startup) => BmsState::Idle,
            (BmsState::Idle, BmsEvent::ChargerConnected) => BmsState::Precharge,
            (BmsState::Precharge, BmsEvent::ChargerConnected) => BmsState::Charging,
            (BmsState::Charging, BmsEvent::ChargerDisconnected) => BmsState::Idle,
            (BmsState::Idle, BmsEvent::Shutdown) => BmsState::Shutdown,
            (_, BmsEvent::CellOverVoltage { .. }) => BmsState::Fault,
            (_, BmsEvent::CellUnderVoltage { .. }) => BmsState::Fault,
            (_, BmsEvent::OverTemperature { .. }) => BmsState::Fault,
            (BmsState::Fault, BmsEvent::FaultCleared) => BmsState::Idle,
            _ => self.current_state, // No transition
        };
    }
}
