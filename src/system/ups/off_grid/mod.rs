pub mod infra;
pub mod physical;
pub mod service;

use crate::signal::signal::Signal;

/// UPS handle with power interfaces for various components.
pub struct UPS<'a> {
    mains: Option<&'a Signal<'a>>,
    battery: Option<&'a Signal<'a>>,
    solar: Option<&'a Signal<'a>>,
    load: Option<&'a Signal<'a>>,
    output: Option<&'a Signal<'a>>,
}

impl<'a> UPS<'a> {
    /// Create a new UPS handle with the given power interfaces.
    pub fn new(
        mains: Option<&'a Signal<'a>>,
        battery: Option<&'a Signal<'a>>,
        solar: Option<&'a Signal<'a>>,
        load: Option<&'a Signal<'a>>,
        output: Option<&'a Signal<'a>>,
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

    pub fn mains(&self) -> Option<&'a Signal<'a>> {
        self.mains
    }

    pub fn battery(&self) -> Option<&'a Signal<'a>> {
        self.battery
    }

    pub fn solar(&self) -> Option<&'a Signal<'a>> {
        self.solar
    }

    pub fn load(&self) -> Option<&'a Signal<'a>> {
        self.load
    }

    pub fn output(&self) -> Option<&'a Signal<'a>> {
        self.output
    }

    /* ---------------- Setters ---------------- */

    pub fn set_mains(&mut self, mains: Option<&'a Signal<'a>>) {
        self.mains = mains;
    }

    pub fn set_battery(&mut self, battery: Option<&'a Signal<'a>>) {
        self.battery = battery;
    }

    pub fn set_solar(&mut self, solar: Option<&'a Signal<'a>>) {
        self.solar = solar;
    }

    pub fn set_load(&mut self, load: Option<&'a Signal<'a>>) {
        self.load = load;
    }

    pub fn set_output(&mut self, output: Option<&'a Signal<'a>>) {
        self.output = output;
    }

    /* -------- Builder-style convenience -------- */

    pub fn with_mains(mut self, mains: &'a Signal<'a>) -> Self {
        self.mains = Some(mains);
        self
    }

    pub fn with_battery(mut self, battery: &'a Signal<'a>) -> Self {
        self.battery = Some(battery);
        self
    }

    pub fn with_solar(mut self, solar: &'a Signal<'a>) -> Self {
        self.solar = Some(solar);
        self
    }

    pub fn with_load(mut self, load: &'a Signal<'a>) -> Self {
        self.load = Some(load);
        self
    }

    pub fn with_output(mut self, output: &'a Signal<'a>) -> Self {
        self.output = Some(output);
        self
    }
}
