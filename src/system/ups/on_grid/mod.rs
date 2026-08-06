mod infra;
mod physical;
mod service;

/// Overall system operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemState {
    Startup,
    DayMode,
    Standby,
    Error,
}

/// Grid frequency status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridFrequencyState {
    NotOk,
    Ok,
}

/// Grid voltage status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridVoltageState {
    NotOk,
    Ok,
}

/// Overall grid health.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridState {
    NotOk,
    Ok,
}

/// PV module status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PvModuleState {
    Undervoltage,
    Overvoltage,
    Ok,
}

/// SCR bridge conduction state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScrBridgeState {
    D4D5Active,
    InactiveSecondQuadrant,
    D3D6Active,
    InactiveFourthQuadrant,
}

/// Fault codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    PvUndervoltage,
    PvOvervoltage,
    GridFrequency,
    GridStartup,
    Grid,
    GridOvercurrent,
    FlybackOvercurrent,
}
