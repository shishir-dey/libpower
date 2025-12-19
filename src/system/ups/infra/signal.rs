/// Generic signal abstraction inspired by Linux signals and
/// FreeRTOS task notifications.
///
/// A signal represents a discrete event, not a continuous value.
/// Signals can be raised, cleared, and polled by consumers.
pub trait Signal {
    /// Identifier for the signal.
    ///
    /// Typically an enum or bitmask.
    type Id: Copy + Eq;

    /// Optional payload associated with the signal.
    ///
    /// Use `()` if no data is needed.
    type Payload;

    /// Raise (send) a signal.
    ///
    /// Can be called from task or ISR context (implementation-defined).
    fn raise(&mut self, id: Self::Id, payload: Self::Payload);

    /// Check if a signal is pending (non-blocking).
    fn is_pending(&self, id: Self::Id) -> bool;

    /// Take (consume) a signal.
    ///
    /// Clears the signal and returns its payload if present.
    fn take(&mut self, id: Self::Id) -> Option<Self::Payload>;

    /// Clear a signal without consuming payload.
    fn clear(&mut self, id: Self::Id);
}
