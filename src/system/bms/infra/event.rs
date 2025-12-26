/// BMS event types for state machine transitions and logging.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BmsEvent {
    /// System startup
    Startup,
    /// Charger connected
    ChargerConnected,
    /// Charger disconnected
    ChargerDisconnected,
    /// Cell overvoltage detected
    CellOverVoltage { cell: usize },
    /// Cell undervoltage detected
    CellUnderVoltage { cell: usize },
    /// Over temperature
    OverTemperature { cell: usize },
    /// Balancing complete
    BalancingComplete,
    /// Fault cleared
    FaultCleared,
    /// Shutdown requested
    Shutdown,
}
