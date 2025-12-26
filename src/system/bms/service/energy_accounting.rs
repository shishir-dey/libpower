/// Energy Accounting
pub trait EnergyAccounting {
    fn coulombs_in(&self) -> f32;
    fn coulombs_out(&self) -> f32;

    fn energy_charged_wh(&self) -> f32;
    fn energy_discharged_wh(&self) -> f32;
}
