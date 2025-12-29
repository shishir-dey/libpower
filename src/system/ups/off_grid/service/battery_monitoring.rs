use super::Service;
use crate::system::ups::off_grid::UPS;
use crate::system::ups::off_grid::infra::event::Event;

/// Battery event types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatteryEvent {
    LowBatteryWarning,
    LowBatteryShutdown,
    HighBattery,
}

/// Battery monitoring service that checks battery voltage and generates events.
pub struct BatteryMonitoringService<'a, E> {
    ups: &'a UPS<'a>,
    event_system: &'a mut E,
    low_warning_threshold: f32,
    low_shutdown_threshold: f32,
    high_threshold: f32,
}

impl<'a, E> BatteryMonitoringService<'a, E>
where
    E: Event<Id = BatteryEvent, Payload = ()>,
{
    /// Create a new battery monitoring service.
    pub fn new(
        ups: &'a UPS<'a>,
        event_system: &'a mut E,
        low_warning_threshold: f32,
        low_shutdown_threshold: f32,
        high_threshold: f32,
    ) -> Self {
        Self {
            ups,
            event_system,
            low_warning_threshold,
            low_shutdown_threshold,
            high_threshold,
        }
    }

    /// Update the battery monitoring logic.
    pub fn update(&mut self) {
        if let Some(battery) = self.ups.battery {
            let voltage = battery.voltage();

            if voltage <= self.low_shutdown_threshold {
                self.event_system
                    .raise(BatteryEvent::LowBatteryShutdown, ());
            } else if voltage <= self.low_warning_threshold {
                self.event_system.raise(BatteryEvent::LowBatteryWarning, ());
            } else if voltage >= self.high_threshold {
                self.event_system.raise(BatteryEvent::HighBattery, ());
            }
        }
    }
}

impl<'a, E> Service for BatteryMonitoringService<'a, E>
where
    E: Event<Id = BatteryEvent, Payload = ()>,
{
    fn update(&mut self) {
        self.update();
    }
}
