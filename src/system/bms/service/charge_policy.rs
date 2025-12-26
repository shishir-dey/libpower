/// Charge Policy
pub trait ChargePolicy {
    fn max_charge_voltage(&self) -> f32;
    fn max_charge_current(&self) -> f32;

    fn max_discharge_current(&self) -> f32;

    fn charge_allowed(&self) -> bool;
    fn discharge_allowed(&self) -> bool;
}
