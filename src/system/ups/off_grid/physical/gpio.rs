use crate::system::ups::off_grid::infra::event::{EdgeEventGenerator, EdgeEventId, Event};

/// Events generated from physical GPIO edges.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioEvent {
    ZeroCrossRising,
    ZeroCrossFalling,
    ShortCircuitRise,
}

impl EdgeEventId for GpioEvent {
    const ZERO_CROSS_RISING: Self = Self::ZeroCrossRising;
    const ZERO_CROSS_FALLING: Self = Self::ZeroCrossFalling;
    const SHORT_CIRCUIT_RISE: Self = Self::ShortCircuitRise;
}

/// Minimal GPIO input pin abstraction.
pub trait GpioInputPin {
    fn is_high(&self) -> bool;
}

/// Mock GPIO input for tests or simulation.
#[derive(Debug, Clone, Copy, Default)]
pub struct GpioInputMock {
    high: bool,
}

impl GpioInputMock {
    pub fn set_high(&mut self, high: bool) {
        self.high = high;
    }
}

impl GpioInputPin for GpioInputMock {
    fn is_high(&self) -> bool {
        self.high
    }
}

/// Thin GPIO wrapper over the generic edge event generator.
#[derive(Debug, Clone, Copy)]
pub struct GpioEventGenerator<Id> {
    generator: EdgeEventGenerator<Id>,
}

impl<Id: EdgeEventId> GpioEventGenerator<Id> {
    pub const fn new() -> Self {
        Self {
            generator: EdgeEventGenerator::new(),
        }
    }

    pub fn reset(&mut self) {
        self.generator.reset();
    }

    pub fn poll<Z, S, E>(&mut self, zero_cross: &Z, short_circuit: &S, events: &mut E)
    where
        Z: GpioInputPin,
        S: GpioInputPin,
        E: Event<Id = Id, Payload = ()>,
    {
        self.generator
            .update_levels(zero_cross.is_high(), short_circuit.is_high(), events);
    }

    pub fn update_levels<E>(
        &mut self,
        zero_cross_high: bool,
        short_circuit_high: bool,
        events: &mut E,
    ) where
        E: Event<Id = Id, Payload = ()>,
    {
        self.generator
            .update_levels(zero_cross_high, short_circuit_high, events);
    }
}

impl GpioEventGenerator<GpioEvent> {
    pub const fn for_io_events() -> Self {
        Self::new()
    }
}

impl Default for GpioEventGenerator<GpioEvent> {
    fn default() -> Self {
        Self::for_io_events()
    }
}
