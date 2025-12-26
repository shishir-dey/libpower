/// Configuration
pub trait Configuration {
    fn chemistry(&self) -> super::super::BatteryChemistry;
    fn set_chemistry(&mut self, chemistry: super::super::BatteryChemistry);

    fn persist(&mut self);
}
