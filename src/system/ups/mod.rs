pub mod infra;
pub mod physical;
pub mod power_interface;
pub mod service;

use crate::system::ups::power_interface::PowerInterface;

/// UPS handle with power interfaces for various components.
pub struct UPS<'a> {
    pub mains: Option<&'a dyn PowerInterface>,
    pub battery: Option<&'a dyn PowerInterface>,
    pub solar: Option<&'a dyn PowerInterface>,
    pub load: Option<&'a dyn PowerInterface>,
    pub output: Option<&'a dyn PowerInterface>,
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
}
