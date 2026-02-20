use core::marker::PhantomData;

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

/// Zero-cost adapter that remaps event IDs from one layer into another.
///
/// This lets a local layer emit local event IDs into a shared outer event bus
/// without coupling the local layer to the outer ID type.
pub struct EventMapper<'a, E, F, LocalId> {
    inner: &'a mut E,
    map: F,
    _local_id: PhantomData<LocalId>,
}

impl<'a, E, F, LocalId> EventMapper<'a, E, F, LocalId> {
    pub fn new(inner: &'a mut E, map: F) -> Self {
        Self {
            inner,
            map,
            _local_id: PhantomData,
        }
    }
}

impl<'a, E, F, LocalId, OuterId> Event for EventMapper<'a, E, F, LocalId>
where
    E: Event<Id = OuterId>,
    F: Fn(LocalId) -> OuterId,
    LocalId: Copy + Eq,
    OuterId: Copy + Eq,
{
    type Id = LocalId;
    type Payload = E::Payload;

    fn raise(&mut self, id: Self::Id, payload: Self::Payload) {
        self.inner.raise((self.map)(id), payload);
    }

    fn is_pending(&self, id: Self::Id) -> bool {
        self.inner.is_pending((self.map)(id))
    }

    fn take(&mut self, id: Self::Id) -> Option<Self::Payload> {
        self.inner.take((self.map)(id))
    }

    fn clear(&mut self, id: Self::Id) {
        self.inner.clear((self.map)(id));
    }

    fn set_hysteresis(&mut self, id: Self::Id, delay_ms: u32) {
        self.inner.set_hysteresis((self.map)(id), delay_ms);
    }
}

/// Extension trait that creates a mapped event view over an event sink.
pub trait EventExt: Event + Sized {
    fn mapped<'a, LocalId, F>(&'a mut self, map: F) -> EventMapper<'a, Self, F, LocalId>
    where
        F: Fn(LocalId) -> Self::Id,
    {
        EventMapper::new(self, map)
    }
}

impl<E: Event> EventExt for E {}

/// Compile-time event IDs used by the GPIO edge event generator.
pub trait EdgeEventId: Copy + Eq {
    const ZERO_CROSS_RISING: Self;
    const ZERO_CROSS_FALLING: Self;
    const SHORT_CIRCUIT_RISE: Self;
}

/// Generic edge event generator driven by sampled GPIO levels.
///
/// It emits:
/// - zero-cross rising and falling edge events
/// - short-circuit rising edge event
#[derive(Debug, Clone, Copy)]
pub struct EdgeEventGenerator<Id> {
    initialized: bool,
    last_zero_cross_high: bool,
    last_short_circuit_high: bool,
    _id: PhantomData<Id>,
}

impl<Id: EdgeEventId> EdgeEventGenerator<Id> {
    pub const fn new() -> Self {
        Self {
            initialized: false,
            last_zero_cross_high: false,
            last_short_circuit_high: false,
            _id: PhantomData,
        }
    }

    pub fn reset(&mut self) {
        self.initialized = false;
        self.last_zero_cross_high = false;
        self.last_short_circuit_high = false;
    }

    pub fn update_levels<E>(
        &mut self,
        zero_cross_high: bool,
        short_circuit_high: bool,
        events: &mut E,
    ) where
        E: Event<Id = Id, Payload = ()>,
    {
        if !self.initialized {
            self.initialized = true;
            self.last_zero_cross_high = zero_cross_high;
            self.last_short_circuit_high = short_circuit_high;
            return;
        }

        if zero_cross_high != self.last_zero_cross_high {
            let event = if zero_cross_high {
                Id::ZERO_CROSS_RISING
            } else {
                Id::ZERO_CROSS_FALLING
            };
            events.raise(event, ());
            self.last_zero_cross_high = zero_cross_high;
        }

        if !self.last_short_circuit_high && short_circuit_high {
            events.raise(Id::SHORT_CIRCUIT_RISE, ());
        }
        self.last_short_circuit_high = short_circuit_high;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum LocalEvent {
        ZeroCross,
        ShortCircuit,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum OuterEvent {
        IoZeroCross,
        IoShortCircuit,
    }

    #[derive(Default)]
    struct TestBus {
        zero_cross_pending: bool,
        short_circuit_pending: bool,
        last_hysteresis: Option<(OuterEvent, u32)>,
    }

    impl Event for TestBus {
        type Id = OuterEvent;
        type Payload = ();

        fn raise(&mut self, id: Self::Id, _payload: Self::Payload) {
            match id {
                OuterEvent::IoZeroCross => self.zero_cross_pending = true,
                OuterEvent::IoShortCircuit => self.short_circuit_pending = true,
            }
        }

        fn is_pending(&self, id: Self::Id) -> bool {
            match id {
                OuterEvent::IoZeroCross => self.zero_cross_pending,
                OuterEvent::IoShortCircuit => self.short_circuit_pending,
            }
        }

        fn take(&mut self, id: Self::Id) -> Option<Self::Payload> {
            let pending = match id {
                OuterEvent::IoZeroCross => &mut self.zero_cross_pending,
                OuterEvent::IoShortCircuit => &mut self.short_circuit_pending,
            };

            if *pending {
                *pending = false;
                Some(())
            } else {
                None
            }
        }

        fn clear(&mut self, id: Self::Id) {
            match id {
                OuterEvent::IoZeroCross => self.zero_cross_pending = false,
                OuterEvent::IoShortCircuit => self.short_circuit_pending = false,
            }
        }

        fn set_hysteresis(&mut self, id: Self::Id, delay_ms: u32) {
            self.last_hysteresis = Some((id, delay_ms));
        }
    }

    fn map_local_to_outer(id: LocalEvent) -> OuterEvent {
        match id {
            LocalEvent::ZeroCross => OuterEvent::IoZeroCross,
            LocalEvent::ShortCircuit => OuterEvent::IoShortCircuit,
        }
    }

    #[test]
    fn event_mapper_translates_all_event_operations() {
        let mut bus = TestBus::default();
        {
            let mut mapped = EventMapper::new(&mut bus, map_local_to_outer);

            mapped.raise(LocalEvent::ZeroCross, ());
            assert!(mapped.is_pending(LocalEvent::ZeroCross));
            assert!(mapped.take(LocalEvent::ZeroCross).is_some());
            assert!(!mapped.is_pending(LocalEvent::ZeroCross));

            mapped.raise(LocalEvent::ShortCircuit, ());
            mapped.clear(LocalEvent::ShortCircuit);
            assert!(!mapped.is_pending(LocalEvent::ShortCircuit));

            mapped.set_hysteresis(LocalEvent::ZeroCross, 25);
        }
        assert_eq!(bus.last_hysteresis, Some((OuterEvent::IoZeroCross, 25)));
    }

    #[test]
    fn event_ext_mapped_creates_mapped_view() {
        let mut bus = TestBus::default();

        {
            let mut mapped = bus.mapped(map_local_to_outer);
            mapped.raise(LocalEvent::ShortCircuit, ());
        }

        assert!(bus.is_pending(OuterEvent::IoShortCircuit));
    }
}
