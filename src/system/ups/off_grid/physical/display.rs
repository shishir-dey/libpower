/// Display control trait and implementations for UPS physical components.
///
/// This module provides a trait for display control with two text buffers (lines),
/// along with a default mock implementation. The display is based on the GLCD
/// system from the assembly code, supporting various messages and conditions.

/// Display message types based on the assembly LCD_DISPLAY functions.
/// For 20x4 LCD display.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisplayMessage {
    /// LCD check (all pixels on)
    LcdCheck,
    /// DSP sine wave display
    DspSine,
    /// System capacity display
    SystemCapacity,
    /// Self test display
    SelfTest,
    /// LCD self test
    LcdSelfTest,
    /// Buzzer test
    BuzzerTest,
    /// Battery voltage and output current
    BatteryVoltageOpCurrent,
    /// Charging current setting
    ChargingCurrentSetting,
    /// Mains present, battery charging
    MainsPresent,
    /// Input voltage and frequency
    InputVoltFreq,
    /// Battery charging indication
    BatteryCharging,
    /// Mains fail, inverter switch off
    MainsFailInvSwitchOff,
    /// Mains fail, switch on inverter
    MainsFailSwitchOnInv,
    /// Mains fail, inverter on
    MainsFailInvOn,
    /// Output load percentage
    OutputLoad,
    /// Attention: battery low
    AttentionBatteryLow,
    /// Remedy for battery low
    RemedyBatteryLow,
    /// Remedy 2 for battery low
    Remedy2BatteryLow,
    /// Protection: short circuit
    ProtectionShort,
    /// Remedy: check wiring
    RemedyCheckWiring,
    /// Battery voltage display
    BatteryVoltage,
    /// Output voltage and frequency
    OutputVoltFreq,
    /// Attention: overload >100%
    AttentionOverload,
    /// Protection: low battery shutdown
    ProtectionLowBattShutdown,
    /// Protection: overload shutdown
    ProtectionOverloadShutdown,
    /// Remedy: reduce some load
    RemedyReduceLoad,
    /// Inverter on
    InverterOn,
    /// Mains fail, inverter off
    MainsFailInvOff,
    /// Battery high cut protection
    BatteryHighCut,
    /// MCB trip
    McbTrip,
    /// High temperature warning
    HighTemperature,
    /// High temperature shutdown
    HighTemperatureShutdown,
}

/// Trait for display control operations with four text buffers for 20x4 LCD.
pub trait Display {
    /// Set the display message.
    fn set_message(&mut self, message: DisplayMessage);

    /// Set the text for line 1 (first buffer).
    /// Text will be truncated to 20 characters.
    fn set_line1(&mut self, text: &str);

    /// Set the text for line 2 (second buffer).
    /// Text will be truncated to 20 characters.
    fn set_line2(&mut self, text: &str);

    /// Set the text for line 3 (third buffer).
    /// Text will be truncated to 20 characters.
    fn set_line3(&mut self, text: &str);

    /// Set the text for line 4 (fourth buffer).
    /// Text will be truncated to 20 characters.
    fn set_line4(&mut self, text: &str);

    /// Get the current text for line 1.
    fn get_line1(&self) -> &str;

    /// Get the current text for line 2.
    fn get_line2(&self) -> &str;

    /// Get the current text for line 3.
    fn get_line3(&self) -> &str;

    /// Get the current text for line 4.
    fn get_line4(&self) -> &str;

    /// Clear all lines.
    fn clear(&mut self) {
        self.set_line1("");
        self.set_line2("");
        self.set_line3("");
        self.set_line4("");
    }
}

/// Mock implementation of the Display trait for testing and default behavior.
#[derive(Debug, Clone)]
pub struct DisplayMock {
    line1: [u8; 20],
    line2: [u8; 20],
    line3: [u8; 20],
    line4: [u8; 20],
}

impl Default for DisplayMock {
    fn default() -> Self {
        Self {
            line1: [b' '; 20],
            line2: [b' '; 20],
            line3: [b' '; 20],
            line4: [b' '; 20],
        }
    }
}

impl Display for DisplayMock {
    fn set_message(&mut self, message: DisplayMessage) {
        let (l1_bytes, l2_bytes, l3_bytes, l4_bytes): (&[u8], &[u8], &[u8], &[u8]) = match message {
            DisplayMessage::LcdCheck => (
                b"                    ",
                b"                    ",
                b"                    ",
                b"                    ",
            ), // All spaces for mock
            DisplayMessage::DspSine => (
                b"DSP Sine Wave       ",
                b"Inverter            ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::SystemCapacity => (
                b"System Capacity     ",
                b"                    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::SelfTest => (
                b"Self Test In        ",
                b"Progress...         ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::LcdSelfTest => (
                b"LCD Self Test       ",
                b"                    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::BuzzerTest => (
                b"Buzzer Test         ",
                b"                    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::BatteryVoltageOpCurrent => (
                b"Battery V O/P C     ",
                b"                    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::ChargingCurrentSetting => (
                b"Charging Current    ",
                b"Setting A           ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::MainsPresent => (
                b"Mains Present       ",
                b"Battery Charging    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::InputVoltFreq => (
                b"I/P Volt I/P Freq   ",
                b"                    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::BatteryCharging => (
                b"                    ",
                b"Battery Charging    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::MainsFailInvSwitchOff => (
                b"Mains Fail          ",
                b"Inverter Switch     ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::MainsFailSwitchOnInv => (
                b"Mains Fail          ",
                b"Switch On Inverter  ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::MainsFailInvOn => (
                b"Mains Fail          ",
                b"Inverter On         ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::OutputLoad => (
                b"O/P Load            ",
                b"                    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::AttentionBatteryLow => (
                b"** Attention **     ",
                b"Battery Low         ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::RemedyBatteryLow => (
                b"** Remedy **        ",
                b"Reduce Load Or      ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::Remedy2BatteryLow => (
                b"** Remedy **        ",
                b"Shutdown Inverter   ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::ProtectionShort => (
                b"** Protection **    ",
                b"Short Ckt/Ovload    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::RemedyCheckWiring => (
                b"** Remedy **        ",
                b"Check Wiring        ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::BatteryVoltage => (
                b"Battery Voltage     ",
                b"                    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::OutputVoltFreq => (
                b"O/P Volt O/P Freq   ",
                b"                    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::AttentionOverload => (
                b"** Attention **     ",
                b"Overload > 100%     ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::ProtectionLowBattShutdown => (
                b"** Protection **    ",
                b"Battery Low         ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::ProtectionOverloadShutdown => (
                b"** Protection **    ",
                b"Overload Shutdown   ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::RemedyReduceLoad => (
                b"** Remedy **        ",
                b"Reduce Some Load    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::InverterOn => (
                b"Mains Fail          ",
                b"Inverter On         ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::MainsFailInvOff => (
                b"Mains Fail          ",
                b"Inverter Off        ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::BatteryHighCut => (
                b"** Protection **    ",
                b"Battery High Cut    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::McbTrip => (
                b"** Protection **    ",
                b"Mains MCB Trip      ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::HighTemperature => (
                b"** Attention **     ",
                b"High Temperature    ",
                b"                    ",
                b"                    ",
            ),
            DisplayMessage::HighTemperatureShutdown => (
                b"High Temperature    ",
                b"Shutdown            ",
                b"                    ",
                b"                    ",
            ),
        };
        let l1_len = l1_bytes.len().min(20);
        self.line1[..l1_len].copy_from_slice(&l1_bytes[..l1_len]);
        for i in l1_len..20 {
            self.line1[i] = b' ';
        }
        let l2_len = l2_bytes.len().min(20);
        self.line2[..l2_len].copy_from_slice(&l2_bytes[..l2_len]);
        for i in l2_len..20 {
            self.line2[i] = b' ';
        }
        let l3_len = l3_bytes.len().min(20);
        self.line3[..l3_len].copy_from_slice(&l3_bytes[..l3_len]);
        for i in l3_len..20 {
            self.line3[i] = b' ';
        }
        let l4_len = l4_bytes.len().min(20);
        self.line4[..l4_len].copy_from_slice(&l4_bytes[..l4_len]);
        for i in l4_len..20 {
            self.line4[i] = b' ';
        }
    }

    fn set_line1(&mut self, text: &str) {
        let bytes = text.as_bytes();
        let len = bytes.len().min(20);
        self.line1[..len].copy_from_slice(&bytes[..len]);
        for i in len..20 {
            self.line1[i] = b' ';
        }
    }

    fn set_line2(&mut self, text: &str) {
        let bytes = text.as_bytes();
        let len = bytes.len().min(20);
        self.line2[..len].copy_from_slice(&bytes[..len]);
        for i in len..20 {
            self.line2[i] = b' ';
        }
    }

    fn set_line3(&mut self, text: &str) {
        let bytes = text.as_bytes();
        let len = bytes.len().min(20);
        self.line3[..len].copy_from_slice(&bytes[..len]);
        for i in len..20 {
            self.line3[i] = b' ';
        }
    }

    fn set_line4(&mut self, text: &str) {
        let bytes = text.as_bytes();
        let len = bytes.len().min(20);
        self.line4[..len].copy_from_slice(&bytes[..len]);
        for i in len..20 {
            self.line4[i] = b' ';
        }
    }

    fn get_line1(&self) -> &str {
        core::str::from_utf8(&self.line1).unwrap_or("")
    }

    fn get_line2(&self) -> &str {
        core::str::from_utf8(&self.line2).unwrap_or("")
    }

    fn get_line3(&self) -> &str {
        core::str::from_utf8(&self.line3).unwrap_or("")
    }

    fn get_line4(&self) -> &str {
        core::str::from_utf8(&self.line4).unwrap_or("")
    }
}
