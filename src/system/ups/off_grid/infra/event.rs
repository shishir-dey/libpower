/// Generic event abstraction inspired by Linux signals and
/// FreeRTOS task notifications.
///
/// An event represents a discrete event, not a continuous value.
/// Events can be raised, cleared, and polled by consumers.
pub trait Event {
    /// Identifier for the event.
    ///
    /// Typically an enum or bitmask.
    type Id: Copy + Eq;

    /// Optional payload associated with the event.
    ///
    /// Use `()` if no data is needed.
    type Payload;

    /// Raise (send) an event.
    ///
    /// Can be called from task or ISR context (implementation-defined).
    fn raise(&mut self, id: Self::Id, payload: Self::Payload);

    /// Check if an event is pending (non-blocking).
    fn is_pending(&self, id: Self::Id) -> bool;

    /// Take (consume) an event.
    ///
    /// Clears the event and returns its payload if present.
    fn take(&mut self, id: Self::Id) -> Option<Self::Payload>;

    /// Clear an event without consuming payload.
    fn clear(&mut self, id: Self::Id);

    /// Set hysteresis delay for an event (in milliseconds).
    ///
    /// The event will only be raised after the delay has passed since the condition was met.
    fn set_hysteresis(&mut self, id: Self::Id, delay_ms: u32);
}
