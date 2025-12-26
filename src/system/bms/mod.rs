pub mod infra;
pub mod physical;
pub mod service;

use physical::{BmsActuation, BmsSensing};
use service::{
    Balancing, ChargePolicy, Configuration, Diagnostics, EnergyAccounting, FaultManagement,
    StateEstimation,
};

/// Battery chemistry families.
/// Models electrochemistry, not vendor SKUs or form factors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum BatteryChemistry {
    /// Lithium Iron Phosphate (LiFePO₄)
    Lfp,

    /// Nickel Manganese Cobalt Oxide (LiNiMnCoO₂)
    Nmc,

    /// Lithium Titanate Oxide (Li₄Ti₅O₁₂ anode)
    Lto,

    /// Generic lithium-ion (unknown or mixed cathode)
    LithiumIon,

    /// Sodium-ion battery (liquid electrolyte)
    SodiumIon,

    /// Solid-state sodium-ion battery
    SolidStateSodiumIon,

    /// All-solid-state lithium battery (ceramic / polymer electrolyte)
    SolidStateLithium,

    /// Semi-solid / gel electrolyte lithium battery
    SemiSolidLithium,

    /// Solid-state lithium metal anode battery
    SolidStateLithiumMetal,

    /// Lithium–Sulfur (Li–S)
    LithiumSulfur,

    /// Lithium–Air (Li–O₂)
    LithiumAir,

    /// Metal–Air batteries (Zn-air, Al-air, Fe-air, etc.)
    MetalAir,

    /// Magnesium-ion battery
    MagnesiumIon,

    /// Aluminum-ion battery
    AluminumIon,

    /// Zinc-based rechargeable battery
    ZincIon,

    /// Redox flow battery (vanadium, iron, organic)
    FlowBattery,

    /// Hybrid battery–supercapacitor system
    HybridCapacitor,
}

/// Fault Model
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BmsFault {
    CellOverVoltage { cell: usize },
    CellUnderVoltage { cell: usize },
    PackOverVoltage,
    PackUnderVoltage,
    OverCurrentCharge,
    OverCurrentDischarge,
    OverTemperature { cell: usize },
    UnderTemperature { cell: usize },
    IsolationFault,
    SensorFault,
    ContactorFault,
    InternalError,
}

/// BMS Trait
pub trait Bms<const N: usize>:
    BmsSensing<N>
    + BmsActuation
    + Balancing<N>
    + StateEstimation
    + EnergyAccounting
    + ChargePolicy
    + FaultManagement
    + Diagnostics
    + Configuration
{
}
