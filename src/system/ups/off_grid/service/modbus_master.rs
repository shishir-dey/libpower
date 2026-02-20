use crate::system::ups::off_grid::physical::serial::Serial;
/// Modbus master implementation for UPS system services.
///
/// This module provides a Modbus RTU master for communicating with devices
/// like MPPT controllers, using the serial interface.
use alloc::boxed::Box;
use heapless::Vec;

const MODBUS_SLAVE_ID: u8 = 1;
const MODBUS_FUNCTION_READ_HOLDING_REGISTERS: u8 = 0x03;
const MODBUS_CRC_POLYNOMIAL: u16 = 0xA001;

/// Modbus communication states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModbusState {
    Idle,
    Transmitting,
    Receiving,
    Processing,
}

/// Modbus request structure.
#[derive(Debug, Clone)]
pub struct ModbusRequest {
    pub slave_id: u8,
    pub function_code: u8,
    pub start_address: u16,
    pub num_registers: u16,
    pub tx_buffer: [u8; 8],
    pub rx_buffer: Vec<u8, 256>,
    pub state: ModbusState,
    pub crc: u16,
}

impl Default for ModbusRequest {
    fn default() -> Self {
        Self {
            slave_id: 0,
            function_code: 0,
            start_address: 0,
            num_registers: 0,
            tx_buffer: [0; 8],
            rx_buffer: Vec::new(),
            state: ModbusState::Idle,
            crc: 0,
        }
    }
}

/// Modbus master structure.
pub struct ModbusMaster {
    serial: Box<dyn Serial>,
    current_request: ModbusRequest,
}

impl ModbusMaster {
    /// Create a new Modbus master with the given serial interface.
    pub fn new(serial: Box<dyn Serial>) -> Self {
        Self {
            serial,
            current_request: ModbusRequest::default(),
        }
    }

    /// Initialize the Modbus master.
    pub fn init(&mut self) {
        self.current_request.state = ModbusState::Idle;
    }

    /// Calculate Modbus CRC16.
    fn modbus_crc16(data: &[u8]) -> u16 {
        let mut crc = 0xFFFFu16;
        for &byte in data {
            crc ^= byte as u16;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ MODBUS_CRC_POLYNOMIAL;
                } else {
                    crc >>= 1;
                }
            }
        }
        crc
    }

    /// Read holding registers from a Modbus slave.
    pub fn read_holding_registers(
        &mut self,
        slave_id: u8,
        start_address: u16,
        num_registers: u16,
    ) -> bool {
        if self.current_request.state != ModbusState::Idle {
            return false;
        }

        self.current_request.slave_id = slave_id;
        self.current_request.function_code = MODBUS_FUNCTION_READ_HOLDING_REGISTERS;
        self.current_request.start_address = start_address;
        self.current_request.num_registers = num_registers;
        self.current_request.state = ModbusState::Transmitting;
        self.current_request.rx_buffer.clear();

        // Build request frame
        let mut tx_buffer = [0u8; 8];
        tx_buffer[0] = slave_id;
        tx_buffer[1] = MODBUS_FUNCTION_READ_HOLDING_REGISTERS;
        tx_buffer[2] = (start_address >> 8) as u8;
        tx_buffer[3] = start_address as u8;
        tx_buffer[4] = (num_registers >> 8) as u8;
        tx_buffer[5] = num_registers as u8;

        let crc = Self::modbus_crc16(&tx_buffer[..6]);
        tx_buffer[6] = crc as u8;
        tx_buffer[7] = (crc >> 8) as u8;

        self.current_request.tx_buffer = tx_buffer;

        // Send
        if self.serial.send(&tx_buffer).is_ok() {
            self.current_request.state = ModbusState::Receiving;
            true
        } else {
            self.current_request.state = ModbusState::Idle;
            false
        }
    }

    /// Process incoming data and handle state transitions.
    pub fn process(&mut self) {
        if self.current_request.state == ModbusState::Receiving {
            // Try to receive data
            let mut temp_buffer = [0u8; 256];
            match self.serial.receive(&mut temp_buffer) {
                Ok(len) => {
                    for &byte in &temp_buffer[..len] {
                        if self.current_request.rx_buffer.len() < 256 {
                            self.current_request.rx_buffer.push(byte).unwrap();
                        }
                    }
                    // For now, just read some bytes and do nothing
                    self.current_request.state = ModbusState::Idle;
                }
                Err(_) => {
                    // Error handling (e.g., timeout)
                }
            }
        }
    }

    /// Take the received data.
    pub fn take_rx_data(&mut self) -> Option<Vec<u8, 256>> {
        if self.current_request.state == ModbusState::Idle
            && !self.current_request.rx_buffer.is_empty()
        {
            let data = self.current_request.rx_buffer.clone();
            self.current_request.rx_buffer.clear();
            Some(data)
        } else {
            None
        }
    }

    /// Get the current request state.
    pub fn get_current_request(&self) -> &ModbusRequest {
        &self.current_request
    }
}
