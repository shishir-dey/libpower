/// Trait for clock functionality providing current time components.
pub trait Clock {
    /// Get the current year.
    fn year(&self) -> u16;

    /// Get the current month (1-12).
    fn month(&self) -> u8;

    /// Get the current day of the month (1-31).
    fn day(&self) -> u8;

    /// Get the current hour (0-23).
    fn hour(&self) -> u8;

    /// Get the current minute (0-59).
    fn minute(&self) -> u8;

    /// Get the current second (0-59).
    fn second(&self) -> u8;

    /// Get the current millisecond (0-999).
    fn millisecond(&self) -> u16;
}
