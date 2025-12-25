/// Power-analyzer–grade abstraction for an electrical power interface.
///
/// Suitable for: mains input, inverter output, battery port,
/// solar input, DC loads, bidirectional converters.
pub trait PowerInterface {
    /// ---------- Basic electrical ----------

    /// Voltage (RMS for AC, DC value for DC)
    fn voltage(&self) -> f32;

    /// Current (RMS for AC, DC value for DC)
    /// Positive = power into the system
    fn current(&self) -> f32;

    /// Frequency in Hz (None for DC)
    fn frequency(&self) -> Option<f32>;

    /// Phase angle between voltage and current (degrees, AC only)
    fn phase_angle(&self) -> Option<f32>;

    /// ---------- Power quantities ----------

    /// Active (real) power P (W)
    fn active_power(&self) -> f32;

    /// Reactive power Q (VAR, AC only)
    fn reactive_power(&self) -> Option<f32>;

    /// Apparent power S (VA, AC only)
    fn apparent_power(&self) -> Option<f32>;

    /// Power factor (true PF, includes harmonics)
    fn power_factor(&self) -> Option<f32>;

    /// Displacement power factor (fundamental only)
    fn displacement_pf(&self) -> Option<f32>;

    /// ---------- Energy ----------

    /// Energy imported into the system (Wh)
    fn energy_imported(&self) -> f64;

    /// Energy exported from the system (Wh)
    fn energy_exported(&self) -> f64;

    /// ---------- Power quality ----------

    /// Voltage THD (%)
    fn voltage_thd(&self) -> Option<f32>;

    /// Current THD (%)
    fn current_thd(&self) -> Option<f32>;

    /// Crest factor of voltage
    fn voltage_crest_factor(&self) -> Option<f32>;

    /// Crest factor of current
    fn current_crest_factor(&self) -> Option<f32>;

    /// Individual harmonic magnitude (n = 1,2,3...)
    /// Returned as percentage of fundamental
    fn voltage_harmonic(&self, n: u8) -> Option<f32>;

    fn current_harmonic(&self, n: u8) -> Option<f32>;

    /// ---------- Efficiency & losses ----------

    /// Conversion efficiency (%) if applicable
    fn efficiency(&self) -> Option<f32>;

    /// Estimated losses (W), if known
    fn losses(&self) -> Option<f32>;

    /// ---------- Limits & ratings ----------

    fn voltage_min(&self) -> f32;
    fn voltage_max(&self) -> f32;
    fn current_max(&self) -> f32;
    fn power_max(&self) -> f32;

    /// ---------- State & health ----------

    fn is_enabled(&self) -> bool;
    fn is_faulted(&self) -> bool;

    /// Interface temperature (°C), if available
    fn temperature(&self) -> Option<f32>;

    /// ---------- Convenience defaults ----------

    /// Instantaneous power derived when analyzer doesn’t provide P directly
    fn instantaneous_power(&self) -> f32 {
        self.voltage() * self.current()
    }
}
