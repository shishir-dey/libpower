/// Sensing Layer (Hardware-Facing)
pub trait BmsSensing<const N: usize> {
    fn cell_voltages(&self) -> &[f32; N];
    fn cell_temperatures(&self) -> &[f32; N];

    fn pack_voltage(&self) -> f32;
    fn pack_current(&self) -> f32; // +charge / -discharge
    fn insulation_resistance_kohm(&self) -> Option<f32>;
}

/// Actuation Layer (Hardware Control)
pub trait BmsActuation {
    fn set_precharge(&mut self, on: bool);
    fn set_charge_contactor(&mut self, on: bool);
    fn set_discharge_contactor(&mut self, on: bool);

    fn request_cooling(&mut self, level: f32); // 0.0–1.0
    fn request_heating(&mut self, level: f32); // 0.0–1.0
}
