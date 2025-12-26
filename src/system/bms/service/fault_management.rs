/// Fault Management
pub trait FaultManagement {
    fn active_faults(&self) -> &[super::super::BmsFault];
    fn has_critical_fault(&self) -> bool;
    fn clear_latched_faults(&mut self);
}
