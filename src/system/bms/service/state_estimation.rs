/// State Estimation
pub trait StateEstimation {
    fn soc(&self) -> f32; // %
    fn soh(&self) -> f32; // %
    fn sop_charge_w(&self) -> f32;
    fn sop_discharge_w(&self) -> f32;

    fn cycle_count(&self) -> u32;
    fn remaining_capacity_ah(&self) -> f32;
}
