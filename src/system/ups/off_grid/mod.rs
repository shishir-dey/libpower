pub mod infra;
pub mod physical;
pub mod power_interface;
pub mod service;

use crate::system::ups::off_grid::power_interface::PowerInterface;

/// UPS handle with power interfaces for various components.
pub struct UPS<'a> {
    mains: Option<&'a dyn PowerInterface>,
    battery: Option<&'a dyn PowerInterface>,
    solar: Option<&'a dyn PowerInterface>,
    load: Option<&'a dyn PowerInterface>,
    output: Option<&'a dyn PowerInterface>,
}

impl<'a> UPS<'a> {
    /// Create a new UPS handle with the given power interfaces.
    pub fn new(
        mains: Option<&'a dyn PowerInterface>,
        battery: Option<&'a dyn PowerInterface>,
        solar: Option<&'a dyn PowerInterface>,
        load: Option<&'a dyn PowerInterface>,
        output: Option<&'a dyn PowerInterface>,
    ) -> Self {
        Self {
            mains,
            battery,
            solar,
            load,
            output,
        }
    }

    /* ---------------- Getters ---------------- */

    pub fn mains(&self) -> Option<&'a dyn PowerInterface> {
        self.mains
    }

    pub fn battery(&self) -> Option<&'a dyn PowerInterface> {
        self.battery
    }

    pub fn solar(&self) -> Option<&'a dyn PowerInterface> {
        self.solar
    }

    pub fn load(&self) -> Option<&'a dyn PowerInterface> {
        self.load
    }

    pub fn output(&self) -> Option<&'a dyn PowerInterface> {
        self.output
    }

    /* ---------------- Setters ---------------- */

    pub fn set_mains(&mut self, mains: Option<&'a dyn PowerInterface>) {
        self.mains = mains;
    }

    pub fn set_battery(&mut self, battery: Option<&'a dyn PowerInterface>) {
        self.battery = battery;
    }

    pub fn set_solar(&mut self, solar: Option<&'a dyn PowerInterface>) {
        self.solar = solar;
    }

    pub fn set_load(&mut self, load: Option<&'a dyn PowerInterface>) {
        self.load = load;
    }

    pub fn set_output(&mut self, output: Option<&'a dyn PowerInterface>) {
        self.output = output;
    }

    /* -------- Builder-style convenience -------- */

    pub fn with_mains(mut self, mains: &'a dyn PowerInterface) -> Self {
        self.mains = Some(mains);
        self
    }

    pub fn with_battery(mut self, battery: &'a dyn PowerInterface) -> Self {
        self.battery = Some(battery);
        self
    }

    pub fn with_solar(mut self, solar: &'a dyn PowerInterface) -> Self {
        self.solar = Some(solar);
        self
    }

    pub fn with_load(mut self, load: &'a dyn PowerInterface) -> Self {
        self.load = Some(load);
        self
    }

    pub fn with_output(mut self, output: &'a dyn PowerInterface) -> Self {
        self.output = Some(output);
        self
    }
}
