//! # Grid-Tied Inverter System
//!
//! This module implements a complete grid-tied photovoltaic inverter system
//! with comprehensive control algorithms and protection features. The system
//! manages power flow from PV panels to the grid while ensuring safe operation
//! and maximum power extraction.
//!
//! ## System Architecture
//!
//! The inverter system consists of:
//! - **State Machine**: Manages operational states and transitions
//! - **MPPT Control**: Tracks maximum power point of PV array
//! - **Grid Synchronization**: SOGI-PLL for robust grid tracking
//! - **Current Control**: PI controller for grid current regulation
//! - **Protection**: Comprehensive fault detection and handling
//!
//! ## Control Strategy
//!
//! 1. **Power Flow Control**: Injects maximum available PV power into grid
//! 2. **Grid Synchronization**: Maintains unity power factor operation
//! 3. **Current Reference**: Generates sinusoidal reference aligned with grid
//! 4. **Voltage Feedforward**: Improves dynamic response and stability
//!
//! ## Safety Features
//!
//! - Grid voltage monitoring (UVLO/OVLO)
//! - Grid frequency monitoring
//! - Anti-islanding protection
//! - PV array protection (voltage/current limits)
//! - Soft-start and controlled shutdown

use crate::control::cntl_pi::ControllerPI;
use crate::mppt::perturb_and_observe::MPPT;
use crate::pll::spll_1ph_sogi::SPLL;
use libm::{fabsf, sinf, sqrtf};

/// System state enumeration for the grid-tied inverter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InverterState {
    /// System is idle, waiting for startup conditions
    Idle,
    /// Checking startup conditions and initializing
    Startup,
    /// Synchronizing with grid using PLL
    PLLSync,
    /// Normal power flow operation
    PowerFlow,
    /// Fault state with specific error condition
    Fault(FaultType),
}

/// Fault types that can occur in the system.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FaultType {
    /// Grid over-voltage condition
    GridOverVoltage,
    /// Grid under-voltage condition
    GridUnderVoltage,
    /// Grid frequency out of range
    GridFrequencyError,
    /// PV over-voltage condition
    PVOverVoltage,
    /// PV under-voltage condition
    PVUnderVoltage,
    /// Output over-current condition
    OverCurrent,
    /// Anti-islanding detection triggered
    Islanding,
    /// Grid connection lost during startup
    GridStartupFault,
}

/// Configuration parameters for the grid-tied inverter.
#[derive(Debug, Clone)]
pub struct InverterConfig {
    /// Nominal grid voltage (RMS) in Volts
    pub grid_voltage_nominal: f32,
    /// Nominal grid frequency in Hz
    pub grid_frequency_nominal: f32,
    /// Maximum output current (peak) in Amperes
    pub max_output_current: f32,
    /// Maximum PV voltage in Volts
    pub pv_voltage_max: f32,
    /// Minimum PV voltage in Volts
    pub pv_voltage_min: f32,
    /// Grid voltage tolerance (±%) for protection
    pub grid_voltage_tolerance: f32,
    /// Grid frequency tolerance (±Hz) for protection
    pub grid_frequency_tolerance: f32,
    /// Control loop sample time in seconds
    pub sample_time: f32,
    /// MPPT update period in seconds
    pub mppt_period: f32,
    /// PLL settling time in seconds
    pub pll_settling_time: f32,
}

impl Default for InverterConfig {
    fn default() -> Self {
        InverterConfig {
            grid_voltage_nominal: 230.0,   // 230V RMS
            grid_frequency_nominal: 50.0,  // 50 Hz
            max_output_current: 20.0,      // 20A peak
            pv_voltage_max: 45.0,          // 45V max
            pv_voltage_min: 15.0,          // 15V min
            grid_voltage_tolerance: 0.1,   // ±10%
            grid_frequency_tolerance: 1.0, // ±1 Hz
            sample_time: 0.0001,           // 100 μs (10 kHz)
            mppt_period: 0.1,              // 100 ms
            pll_settling_time: 0.5,        // 500 ms
        }
    }
}

/// Measurement data structure for system inputs.
#[derive(Debug, Clone, Copy)]
pub struct Measurements {
    /// PV array voltage in Volts
    pub pv_voltage: f32,
    /// PV array current in Amperes
    pub pv_current: f32,
    /// Grid voltage (instantaneous) in Volts
    pub grid_voltage: f32,
    /// Output current (instantaneous) in Amperes
    pub output_current: f32,
}

impl Default for Measurements {
    fn default() -> Self {
        Measurements {
            pv_voltage: 0.0,
            pv_current: 0.0,
            grid_voltage: 0.0,
            output_current: 0.0,
        }
    }
}

/// Control outputs for the power stage.
#[derive(Debug, Clone, Copy)]
pub struct ControlOutputs {
    /// PWM duty cycle (0.0 to 1.0)
    pub duty_cycle: f32,
    /// Grid relay control (true = closed)
    pub grid_relay: bool,
    /// Status LED states
    pub led_power: bool,
    pub led_fault: bool,
}

impl Default for ControlOutputs {
    fn default() -> Self {
        ControlOutputs {
            duty_cycle: 0.0,
            grid_relay: false,
            led_power: false,
            led_fault: false,
        }
    }
}

/// Grid-tied inverter system implementation.
///
/// This structure encapsulates all control algorithms and state management
/// for a complete grid-tied PV inverter system.
///
/// # Examples
///
/// ```rust
/// use libpower::system::on_grid::{GridTieInverter, InverterConfig, Measurements};
///
/// // Create inverter with default configuration
/// let config = InverterConfig::default();
/// let mut inverter = GridTieInverter::new(config);
///
/// // Update with measurements
/// let measurements = Measurements {
///     pv_voltage: 35.0,
///     pv_current: 8.0,
///     grid_voltage: 325.0 * (0.0f32).sin(), // 230V RMS sine wave
///     output_current: 0.0,
/// };
///
/// // Run control loop
/// let outputs = inverter.update(measurements);
///
/// // Check system state
/// println!("State: {:?}", inverter.get_state());
/// println!("Duty: {:.2}%", outputs.duty_cycle * 100.0);
/// ```
pub struct GridTieInverter {
    // Configuration
    config: InverterConfig,

    // State machine
    state: InverterState,
    state_timer: f32,

    // Control modules
    mppt: MPPT,
    pll: SPLL,
    current_controller: ControllerPI,

    // Internal variables
    measurements: Measurements,
    grid_voltage_peak: f32,
    grid_frequency: f32,
    current_reference: f32,
    voltage_reference: f32,
    power_limit: f32,

    // Timers and counters
    time_elapsed: f32,
    mppt_timer: f32,
    startup_counter: u32,
    fault_counter: u32,

    // Protection flags
    grid_voltage_ok: bool,
    grid_frequency_ok: bool,
    pv_voltage_ok: bool,
    anti_islanding_ok: bool,
}

impl GridTieInverter {
    /// Creates a new grid-tied inverter with the specified configuration.
    ///
    /// # Parameters
    ///
    /// * `config` - System configuration parameters
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::system::on_grid::{GridTieInverter, InverterConfig};
    ///
    /// let mut config = InverterConfig::default();
    /// config.grid_voltage_nominal = 120.0;  // 120V RMS for US grid
    /// config.grid_frequency_nominal = 60.0; // 60 Hz
    ///
    /// let inverter = GridTieInverter::new(config);
    /// ```
    pub fn new(config: InverterConfig) -> Self {
        // Initialize MPPT
        let mut mppt = MPPT::new();
        mppt.set_step_size(0.5); // 0.5V steps
        mppt.set_mppt_v_out_max(config.pv_voltage_max);
        mppt.set_mppt_v_out_min(config.pv_voltage_min);

        // Initialize PLL
        let pll = SPLL::new(config.grid_frequency_nominal, 1.0 / config.sample_time);

        // Initialize current controller
        let mut current_controller = ControllerPI::new();
        current_controller.set_gains(0.5, 100.0); // Kp=0.5, Ki=100
        current_controller.set_limits(0.0, 0.95); // Duty cycle limits

        GridTieInverter {
            config,
            state: InverterState::Idle,
            state_timer: 0.0,
            mppt,
            pll,
            current_controller,
            measurements: Measurements::default(),
            grid_voltage_peak: 0.0,
            grid_frequency: 0.0,
            current_reference: 0.0,
            voltage_reference: 0.0,
            power_limit: 0.0,
            time_elapsed: 0.0,
            mppt_timer: 0.0,
            startup_counter: 0,
            fault_counter: 0,
            grid_voltage_ok: false,
            grid_frequency_ok: false,
            pv_voltage_ok: false,
            anti_islanding_ok: true,
        }
    }

    /// Gets the current system state.
    ///
    /// # Returns
    ///
    /// The current `InverterState`.
    pub fn get_state(&self) -> InverterState {
        self.state
    }

    /// Gets the current grid frequency estimate.
    ///
    /// # Returns
    ///
    /// Estimated grid frequency in Hz.
    pub fn get_grid_frequency(&self) -> f32 {
        self.grid_frequency
    }

    /// Gets the current power being processed.
    ///
    /// # Returns
    ///
    /// Instantaneous power in Watts.
    pub fn get_power(&self) -> f32 {
        self.measurements.pv_voltage * self.measurements.pv_current
    }

    /// Main update function for the inverter control system.
    ///
    /// This method should be called at the configured sample rate with fresh
    /// measurement data. It executes all control algorithms and state machine
    /// logic, returning the appropriate control outputs.
    ///
    /// # Parameters
    ///
    /// * `measurements` - Current system measurements
    ///
    /// # Returns
    ///
    /// Control outputs for the power stage and indicators.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::system::on_grid::{GridTieInverter, InverterConfig, Measurements};
    ///
    /// let mut inverter = GridTieInverter::new(InverterConfig::default());
    ///
    /// // Simulation loop
    /// for i in 0..1000 {
    ///     let t = i as f32 * 0.0001; // 100μs steps
    ///     
    ///     let measurements = Measurements {
    ///         pv_voltage: 35.0,
    ///         pv_current: 8.0,
    ///         grid_voltage: 325.0 * (2.0 * std::f32::consts::PI * 50.0 * t).sin(),
    ///         output_current: 10.0 * (2.0 * std::f32::consts::PI * 50.0 * t).sin(),
    ///     };
    ///     
    ///     let outputs = inverter.update(measurements);
    /// }
    /// ```
    pub fn update(&mut self, measurements: Measurements) -> ControlOutputs {
        // Store measurements
        self.measurements = measurements;

        // Update timers
        self.time_elapsed += self.config.sample_time;
        self.state_timer += self.config.sample_time;
        self.mppt_timer += self.config.sample_time;

        // Run protection checks
        self.check_protections();

        // Execute state machine
        match self.state {
            InverterState::Idle => self.state_idle(),
            InverterState::Startup => self.state_startup(),
            InverterState::PLLSync => self.state_pll_sync(),
            InverterState::PowerFlow => self.state_power_flow(),
            InverterState::Fault(_) => self.state_fault(),
        }

        // Generate control outputs based on state
        self.generate_outputs()
    }

    /// Checks all protection conditions and updates status flags.
    fn check_protections(&mut self) {
        // Grid voltage protection
        let grid_v_rms = self.grid_voltage_peak / core::f32::consts::SQRT_2;
        let v_min = self.config.grid_voltage_nominal * (1.0 - self.config.grid_voltage_tolerance);
        let v_max = self.config.grid_voltage_nominal * (1.0 + self.config.grid_voltage_tolerance);

        self.grid_voltage_ok = grid_v_rms >= v_min && grid_v_rms <= v_max;

        // Grid frequency protection
        let f_min = self.config.grid_frequency_nominal - self.config.grid_frequency_tolerance;
        let f_max = self.config.grid_frequency_nominal + self.config.grid_frequency_tolerance;

        self.grid_frequency_ok = self.grid_frequency >= f_min && self.grid_frequency <= f_max;

        // PV voltage protection
        self.pv_voltage_ok = self.measurements.pv_voltage >= self.config.pv_voltage_min
            && self.measurements.pv_voltage <= self.config.pv_voltage_max;

        // Anti-islanding (simplified - checks for grid presence)
        self.anti_islanding_ok = self.grid_voltage_peak > 50.0; // Simple threshold

        // Check for fault conditions during normal operation
        if self.state == InverterState::PowerFlow {
            if !self.grid_voltage_ok {
                if grid_v_rms > v_max {
                    self.transition_to_fault(FaultType::GridOverVoltage);
                } else {
                    self.transition_to_fault(FaultType::GridUnderVoltage);
                }
            } else if !self.grid_frequency_ok {
                self.transition_to_fault(FaultType::GridFrequencyError);
            } else if !self.pv_voltage_ok {
                if self.measurements.pv_voltage > self.config.pv_voltage_max {
                    self.transition_to_fault(FaultType::PVOverVoltage);
                } else {
                    self.transition_to_fault(FaultType::PVUnderVoltage);
                }
            } else if !self.anti_islanding_ok {
                self.transition_to_fault(FaultType::Islanding);
            } else if fabsf(self.measurements.output_current) > self.config.max_output_current {
                self.transition_to_fault(FaultType::OverCurrent);
            }
        }
    }

    /// Idle state handler - waiting for valid startup conditions.
    fn state_idle(&mut self) {
        // Check if we can start up
        if self.grid_voltage_ok && self.pv_voltage_ok {
            self.transition_to_state(InverterState::Startup);
        }
    }

    /// Startup state handler - system initialization and checks.
    fn state_startup(&mut self) {
        // Reset controllers
        if self.state_timer < 0.01 {
            // First entry
            self.current_controller.reset();
            self.pll.reset();
            self.startup_counter = 0;
        }

        // Check grid conditions for multiple cycles
        if self.grid_voltage_ok && self.grid_frequency_ok {
            self.startup_counter += 1;

            // Require 100 consecutive good samples (~10ms at 10kHz)
            if self.startup_counter > 100 {
                self.transition_to_state(InverterState::PLLSync);
            }
        } else {
            self.startup_counter = 0;

            // Timeout after 1 second
            if self.state_timer > 1.0 {
                self.transition_to_fault(FaultType::GridStartupFault);
            }
        }
    }

    /// PLL synchronization state handler.
    fn state_pll_sync(&mut self) {
        // Run PLL
        self.pll.calculate(self.measurements.grid_voltage);
        self.grid_frequency = self.pll.get_frequency();

        // Update peak voltage estimate
        let v_alpha = self.pll.get_v_alpha();
        let v_beta = self.pll.get_v_beta();
        self.grid_voltage_peak = sqrtf(v_alpha * v_alpha + v_beta * v_beta);

        // Check if PLL has settled
        if self.state_timer > self.config.pll_settling_time {
            if self.grid_frequency_ok {
                // Initialize MPPT with current PV voltage
                self.mppt
                    .calculate(self.measurements.pv_current, self.measurements.pv_voltage);
                self.voltage_reference = self.measurements.pv_voltage;

                self.transition_to_state(InverterState::PowerFlow);
            }
        }
    }

    /// Power flow state handler - normal operation.
    fn state_power_flow(&mut self) {
        // Run PLL
        self.pll.calculate(self.measurements.grid_voltage);
        self.grid_frequency = self.pll.get_frequency();

        // Update peak voltage
        let v_alpha = self.pll.get_v_alpha();
        let v_beta = self.pll.get_v_beta();
        self.grid_voltage_peak = sqrtf(v_alpha * v_alpha + v_beta * v_beta);

        // Run MPPT at slower rate
        if self.mppt_timer >= self.config.mppt_period {
            self.mppt_timer = 0.0;
            self.mppt
                .calculate(self.measurements.pv_current, self.measurements.pv_voltage);
            self.voltage_reference = self.mppt.get_mppt_v_out();
        }

        // Calculate power limit based on PV
        let pv_power = self.measurements.pv_voltage * self.measurements.pv_current;
        self.power_limit = pv_power * 0.95; // 95% efficiency assumption

        // Generate current reference (sinusoidal, in phase with grid)
        let phase = self.pll.get_phase();
        let i_peak = (2.0 * self.power_limit) / self.grid_voltage_peak;
        self.current_reference = i_peak * sinf(phase);

        // Limit current reference
        if self.current_reference > self.config.max_output_current {
            self.current_reference = self.config.max_output_current;
        } else if self.current_reference < -self.config.max_output_current {
            self.current_reference = -self.config.max_output_current;
        }
    }

    /// Fault state handler.
    fn state_fault(&mut self) {
        // Stay in fault for minimum time
        if self.state_timer > 2.0 {
            // Check if fault condition cleared
            match self.state {
                InverterState::Fault(fault_type) => {
                    let can_recover = match fault_type {
                        FaultType::GridOverVoltage | FaultType::GridUnderVoltage => {
                            self.grid_voltage_ok
                        }
                        FaultType::GridFrequencyError => self.grid_frequency_ok,
                        FaultType::PVOverVoltage | FaultType::PVUnderVoltage => self.pv_voltage_ok,
                        _ => true,
                    };

                    if can_recover {
                        self.transition_to_state(InverterState::Idle);
                    }
                }
                _ => {}
            }
        }
    }

    /// Generates control outputs based on current state.
    fn generate_outputs(&mut self) -> ControlOutputs {
        let mut outputs = ControlOutputs::default();

        match self.state {
            InverterState::Idle => {
                outputs.duty_cycle = 0.0;
                outputs.grid_relay = false;
                outputs.led_power = false;
                outputs.led_fault = false;
            }
            InverterState::Startup => {
                outputs.duty_cycle = 0.0;
                outputs.grid_relay = false;
                outputs.led_power = true;
                outputs.led_fault = false;
            }
            InverterState::PLLSync => {
                outputs.duty_cycle = 0.0;
                outputs.grid_relay = true; // Close relay but no power yet
                outputs.led_power = true;
                outputs.led_fault = false;
            }
            InverterState::PowerFlow => {
                // Run current controller
                let duty = self
                    .current_controller
                    .calculate(self.current_reference, self.measurements.output_current);

                // Apply feedforward term
                let feedforward = if self.measurements.pv_voltage > 0.0 {
                    fabsf(self.measurements.grid_voltage) / self.measurements.pv_voltage
                } else {
                    0.0
                };

                outputs.duty_cycle = (duty + feedforward).min(0.95).max(0.0);
                outputs.grid_relay = true;
                outputs.led_power = true;
                outputs.led_fault = false;
            }
            InverterState::Fault(_) => {
                outputs.duty_cycle = 0.0;
                outputs.grid_relay = false;
                outputs.led_power = false;
                outputs.led_fault = true;
            }
        }

        outputs
    }

    /// Transitions to a new state.
    fn transition_to_state(&mut self, new_state: InverterState) {
        self.state = new_state;
        self.state_timer = 0.0;
    }

    /// Transitions to fault state with specific fault type.
    fn transition_to_fault(&mut self, fault_type: FaultType) {
        self.state = InverterState::Fault(fault_type);
        self.state_timer = 0.0;
        self.fault_counter += 1;
    }

    /// Resets the system to idle state.
    ///
    /// This method can be used to manually reset the system after a fault
    /// or for testing purposes.
    pub fn reset(&mut self) {
        self.state = InverterState::Idle;
        self.state_timer = 0.0;
        self.current_controller.reset();
        self.pll.reset();
        self.startup_counter = 0;
        self.fault_counter = 0;
    }

    /// Gets diagnostic information about the system.
    ///
    /// # Returns
    ///
    /// A tuple containing various system parameters for monitoring.
    pub fn get_diagnostics(&self) -> (f32, f32, f32, f32, u32) {
        (
            self.grid_voltage_peak / core::f32::consts::SQRT_2, // Grid RMS voltage
            self.grid_frequency,                                // Grid frequency
            self.get_power(),                                   // PV power
            self.voltage_reference,                             // MPPT voltage reference
            self.fault_counter,                                 // Fault count
        )
    }
}
