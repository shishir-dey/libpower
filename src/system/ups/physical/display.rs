/// Display control trait and implementations for UPS physical components.
///
/// This module provides a trait for display control with two text buffers (lines),
/// along with a default mock implementation. The display is based on the GLCD
/// system from the assembly code, supporting various messages and conditions.

/// Display message types based on the assembly LCD_DISPLAY functions.
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

/// Trait for display control operations with two text buffers.
pub trait Display {
    /// Set the display message.
    fn set_message(&mut self, message: DisplayMessage);

    /// Set the text for line 1 (first buffer).
    /// Text will be truncated to 16 characters.
    fn set_line1(&mut self, text: &str);

    /// Set the text for line 2 (second buffer).
    /// Text will be truncated to 16 characters.
    fn set_line2(&mut self, text: &str);

    /// Get the current text for line 1.
    fn get_line1(&self) -> &str;

    /// Get the current text for line 2.
    fn get_line2(&self) -> &str;

    /// Clear both lines.
    fn clear(&mut self) {
        self.set_line1("");
        self.set_line2("");
    }
}

/// Mock implementation of the Display trait for testing and default behavior.
#[derive(Debug, Clone, Default)]
pub struct DisplayMock {
    line1: String,
    line2: String,
}

impl Display for DisplayMock {
    fn set_message(&mut self, message: DisplayMessage) {
        let (line1, line2) = match message {
            DisplayMessage::LcdCheck => (
                "                ".to_string(),
                "                ".to_string(),
            ), // All spaces for mock
            DisplayMessage::DspSine => (
                "DSP Sine Wave   ".to_string(),
                "Inverter        ".to_string(),
            ),
            DisplayMessage::SystemCapacity => (
                "System Capacity ".to_string(),
                "                ".to_string(),
            ),
            DisplayMessage::SelfTest => (
                "Self Test In    ".to_string(),
                "Progress...     ".to_string(),
            ),
            DisplayMessage::LcdSelfTest => (
                "LCD Self Test   ".to_string(),
                "                ".to_string(),
            ),
            DisplayMessage::BuzzerTest => (
                "Buzzer Test     ".to_string(),
                "                ".to_string(),
            ),
            DisplayMessage::BatteryVoltageOpCurrent => (
                "Battery V O/P C ".to_string(),
                "                ".to_string(),
            ),
            DisplayMessage::ChargingCurrentSetting => (
                "Charging Current".to_string(),
                "Setting A       ".to_string(),
            ),
            DisplayMessage::MainsPresent => (
                "Mains Present   ".to_string(),
                "Battery Charging".to_string(),
            ),
            DisplayMessage::InputVoltFreq => (
                "I/P Volt I/P Freq".to_string(),
                "                ".to_string(),
            ),
            DisplayMessage::BatteryCharging => (
                "                ".to_string(),
                "Battery Charging".to_string(),
            ),
            DisplayMessage::MainsFailInvSwitchOff => (
                "Mains Fail      ".to_string(),
                "Inverter Switch ".to_string(),
            ),
            DisplayMessage::MainsFailSwitchOnInv => (
                "Mains Fail      ".to_string(),
                "Switch On Inverter".to_string(),
            ),
            DisplayMessage::MainsFailInvOn => (
                "Mains Fail      ".to_string(),
                "Inverter On     ".to_string(),
            ),
            DisplayMessage::OutputLoad => (
                "O/P Load        ".to_string(),
                "                ".to_string(),
            ),
            DisplayMessage::AttentionBatteryLow => (
                "** Attention ** ".to_string(),
                "Battery Low     ".to_string(),
            ),
            DisplayMessage::RemedyBatteryLow => (
                "** Remedy **    ".to_string(),
                "Reduce Load Or  ".to_string(),
            ),
            DisplayMessage::Remedy2BatteryLow => (
                "** Remedy **    ".to_string(),
                "Shutdown Inverter".to_string(),
            ),
            DisplayMessage::ProtectionShort => (
                "** Protection **".to_string(),
                "Short Ckt/Ovload".to_string(),
            ),
            DisplayMessage::RemedyCheckWiring => (
                "** Remedy **    ".to_string(),
                "Check Wiring    ".to_string(),
            ),
            DisplayMessage::BatteryVoltage => (
                "Battery Voltage ".to_string(),
                "                ".to_string(),
            ),
            DisplayMessage::OutputVoltFreq => (
                "O/P Volt O/P Freq".to_string(),
                "                ".to_string(),
            ),
            DisplayMessage::AttentionOverload => (
                "** Attention ** ".to_string(),
                "Overload > 100% ".to_string(),
            ),
            DisplayMessage::ProtectionLowBattShutdown => (
                "** Protection **".to_string(),
                "Battery Low     ".to_string(),
            ),
            DisplayMessage::ProtectionOverloadShutdown => (
                "** Protection **".to_string(),
                "Overload Shutdown".to_string(),
            ),
            DisplayMessage::RemedyReduceLoad => (
                "** Remedy **    ".to_string(),
                "Reduce Some Load".to_string(),
            ),
            DisplayMessage::InverterOn => (
                "Mains Fail      ".to_string(),
                "Inverter On     ".to_string(),
            ),
            DisplayMessage::MainsFailInvOff => (
                "Mains Fail      ".to_string(),
                "Inverter Off    ".to_string(),
            ),
            DisplayMessage::BatteryHighCut => (
                "** Protection **".to_string(),
                "Battery High Cut".to_string(),
            ),
            DisplayMessage::McbTrip => (
                "** Protection **".to_string(),
                "Mains MCB Trip  ".to_string(),
            ),
            DisplayMessage::HighTemperature => (
                "** Attention ** ".to_string(),
                "High Temperature".to_string(),
            ),
            DisplayMessage::HighTemperatureShutdown => (
                "High Temperature".to_string(),
                "Shutdown        ".to_string(),
            ),
        };
        self.line1 = format!("{:<16}", line1).chars().take(16).collect();
        self.line2 = format!("{:<16}", line2).chars().take(16).collect();
    }

    fn set_line1(&mut self, text: &str) {
        self.line1 = text.chars().take(16).collect();
    }

    fn set_line2(&mut self, text: &str) {
        self.line2 = text.chars().take(16).collect();
    }

    fn get_line1(&self) -> &str {
        &self.line1
    }

    fn get_line2(&self) -> &str {
        &self.line2
    }
}
