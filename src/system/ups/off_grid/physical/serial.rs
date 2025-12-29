/// Minimal serial communication trait and mock implementation for UPS physical components.
///
/// This module provides a basic serial interface inspired by USART drivers,
/// with send and receive operations for byte-level communication.

/// A generic error type for serial operations, using i32 for error codes.
pub type Error = i32;

/// A specialized Result type for serial operations.
pub type Result<T> = core::result::Result<T, Error>;

/// Minimal trait for serial communication operations.
pub trait Serial {
    /// Send data over the serial interface.
    fn send(&mut self, data: &[u8]) -> Result<()>;

    /// Receive data from the serial interface into the provided buffer.
    /// Returns the number of bytes received.
    fn receive(&mut self, buffer: &mut [u8]) -> Result<usize>;
}

/// Mock implementation of the Serial trait for testing and default behavior.
#[derive(Debug, Clone, Default)]
pub struct SerialMock;

impl Serial for SerialMock {
    fn send(&mut self, _data: &[u8]) -> Result<()> {
        // Mock implementation: always succeed
        Ok(())
    }

    fn receive(&mut self, buffer: &mut [u8]) -> Result<usize> {
        // Mock implementation: no data received
        for byte in buffer.iter_mut() {
            *byte = 0;
        }
        Ok(0)
    }
}
