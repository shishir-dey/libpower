/// Diagnostics
pub trait Diagnostics {
    fn firmware_version(&self) -> &'static str;
    fn uptime_seconds(&self) -> u64;

    fn fault_history(&self) -> &[super::super::BmsFault];
}
