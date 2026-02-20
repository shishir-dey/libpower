//! # Signal Generation and Analysis Module
//!
//! This module provides comprehensive signal generation and analysis capabilities
//! for power electronics applications. It supports various waveform types commonly
//! encountered in power systems and includes detailed signal metrics computation.
//!
//! ## Supported Signal Types
//!
//! - **DC**: Constant voltage/current signals
//! - **Sine**: Fundamental AC waveforms
//! - **Full-wave rectified sine**: Used in DC power supplies
//! - **Half-wave rectified sine**: Simple rectifier outputs
//! - **PWM**: Pulse Width Modulation signals for switching converters
//! - **Triangular**: Carrier waves for PWM generation
//! - **Square**: Digital control signals and switching waveforms
//! - **Sawtooth**: Linear ramp signals for timing and control
//!
//! ## Signal Metrics
//!
//! The module computes comprehensive signal quality metrics:
//! - Peak-to-peak, maximum, minimum values
//! - Average (DC component) and RMS values
//! - Duty cycle analysis (positive and negative)
//! - Rise and fall time measurements
//! - Crest factor and THD (Total Harmonic Distortion)
//!
//! ## Unified Electrical Interface
//!
//! The [`Signal`] struct also models electrical port parameters used by power
//! systems, such as voltage/current, power, energy, limits, and health status.
//! This keeps waveform generation and electrical interface analysis in one
//! concrete type.
//!
//! ## Applications
//!
//! - Power quality analysis
//! - Converter waveform generation
//! - System testing and validation
//! - Control signal synthesis
//! - Harmonic analysis

use core::f32::consts::PI;

extern crate libm; // For floating point math operations in no_std

/// Signal type enumeration for waveform generation.
///
/// Each variant represents a different mathematical function used
/// to generate time-domain waveforms commonly found in power electronics.
#[derive(Debug, Clone, PartialEq)]
pub enum SignalType {
    /// Constant DC signal: f(t) = amplitude
    DC,
    /// Sinusoidal signal: f(t) = amplitude × sin(2πft + φ)
    Sine,
    /// Full-wave rectified sine: f(t) = amplitude × |sin(2πft + φ)|
    FullWaveRectifiedSine,
    /// Half-wave rectified sine: f(t) = amplitude × max(0, sin(2πft + φ))
    HalfWaveRectifiedSine,
    /// PWM signal with configurable duty cycle
    PWM,
    /// Triangular wave: Linear ramp up and down
    Triangular,
    /// Square wave: Alternating high and low levels
    Square,
    /// Sawtooth wave: Linear ramp with sharp reset
    Sawtooth,
}

/// Window function type for spectral analysis.
///
/// Window functions reduce spectral leakage when performing DFT analysis
/// on non-periodic sample windows. Each type offers a different trade-off
/// between main-lobe width (frequency resolution) and side-lobe level
/// (spectral leakage).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WindowType {
    /// No windowing — equivalent to a rectangular window.
    Rectangular,
    /// Hanning (Hann) window: good general-purpose choice.
    /// w(n) = 0.5 × (1 − cos(2πn / (N−1)))
    Hanning,
    /// Hamming window: slightly higher side-lobe rejection than Hanning.
    /// w(n) = 0.54 − 0.46 × cos(2πn / (N−1))
    Hamming,
    /// Blackman window: very low side-lobes at the expense of wider main lobe.
    /// w(n) = 0.42 − 0.5 × cos(2πn / (N−1)) + 0.08 × cos(4πn / (N−1))
    Blackman,
    /// Flat-top window: best amplitude accuracy for single-tone measurements.
    /// Uses the SFT5 coefficients.
    FlatTop,
}

/// Signal generator and analyzer for power electronics waveforms.
///
/// This struct provides both signal generation capabilities and comprehensive
/// signal analysis including quality metrics computation. It operates on
/// a provided sample buffer to maintain `no_std` compatibility.
///
/// # Examples
///
/// ## Sine Wave Generation
///
/// ```rust
/// use libpower::signal::signal::{Signal, SignalType};
///
/// let mut buffer = [0.0; 1000];
/// let mut signal = Signal::new(SignalType::Sine, 10.0, 50.0, &mut buffer);
/// signal.set_phase(0.0);
///
/// let samples = signal.get_samples();
/// let metrics = signal.get_metrics();
///
/// // Verify sine wave properties
/// assert!((metrics.dc_rms - 7.07).abs() < 0.1); // ~10/√2
/// assert!(metrics.average.abs() < 0.1); // Zero average for pure sine
/// ```
///
/// ## PWM Signal Analysis
///
/// ```rust
/// use libpower::signal::signal::{Signal, SignalType};
///
/// let mut buffer = [0.0; 1000];
/// let mut signal = Signal::new(SignalType::PWM, 5.0, 1000.0, &mut buffer);
/// signal.set_duty_cycle(0.25); // 25% duty cycle
///
/// let metrics = signal.get_metrics();
/// assert!((metrics.duty_cycle_pos - 0.25).abs() < 0.01);
/// assert!((metrics.average - 1.25).abs() < 0.1); // 25% of 5V
/// ```
///
/// ## Signal Quality Metrics
///
/// ```rust
/// use libpower::signal::signal::{Signal, SignalType};
///
/// let mut buffer = [0.0; 500];
/// let mut signal = Signal::new(SignalType::Square, 12.0, 60.0, &mut buffer);
///
/// let metrics = signal.get_metrics();
/// assert_eq!(metrics.max, 12.0);
/// assert_eq!(metrics.min, -12.0);
/// assert_eq!(metrics.peak_to_peak, 24.0);
/// assert!((metrics.crest_factor - 1.0).abs() < 0.1); // Square wave has crest factor ≈ 1.0
/// ```
pub struct Signal<'a> {
    /// Type of waveform to generate
    wave_type: SignalType,
    /// Peak amplitude of the signal
    amplitude: f32,
    /// Phase offset in degrees
    phase: f32,
    /// Fundamental frequency in Hz
    frequency: f32,
    /// Number of samples in the buffer
    num_samples: u32,
    /// Duty cycle for PWM signals (0.0 to 1.0)
    duty_cycle: f32,
    /// Port voltage in volts
    voltage: f32,
    /// Port current in amps
    current: f32,
    /// Port frequency in hertz (`None` for DC interfaces)
    port_frequency: Option<f32>,
    /// Port phase angle between voltage and current in degrees
    phase_angle: Option<f32>,
    /// Active (real) power in watts
    active_power: f32,
    /// Reactive power in vars
    reactive_power: Option<f32>,
    /// Apparent power in volt-amps
    apparent_power: Option<f32>,
    /// True power factor
    power_factor: Option<f32>,
    /// Displacement power factor
    displacement_pf: Option<f32>,
    /// Energy imported in watt-hours
    energy_imported: f64,
    /// Energy exported in watt-hours
    energy_exported: f64,
    /// Voltage THD in percent
    voltage_thd: Option<f32>,
    /// Current THD in percent
    current_thd: Option<f32>,
    /// Voltage crest factor
    voltage_crest_factor: Option<f32>,
    /// Current crest factor
    current_crest_factor: Option<f32>,
    /// Voltage harmonic magnitudes as percent of fundamental (index = harmonic number)
    voltage_harmonics: [Option<f32>; 16],
    /// Current harmonic magnitudes as percent of fundamental (index = harmonic number)
    current_harmonics: [Option<f32>; 16],
    /// Conversion efficiency in percent
    efficiency: Option<f32>,
    /// Estimated losses in watts
    losses: Option<f32>,
    /// Minimum supported voltage in volts
    voltage_min: f32,
    /// Maximum supported voltage in volts
    voltage_max: f32,
    /// Maximum supported current in amps
    current_max: f32,
    /// Maximum supported power in watts
    power_max: f32,
    /// Whether the interface is enabled
    is_enabled: bool,
    /// Whether the interface is faulted
    is_faulted: bool,
    /// Interface temperature in degrees Celsius
    temperature: Option<f32>,
    // Internal measurements
    /// Peak-to-peak amplitude
    peak_to_peak: f32,
    /// Maximum sample value
    max: f32,
    /// Minimum sample value
    min: f32,
    /// Maximum positive excursion from average
    positive_peak: f32,
    /// Maximum negative excursion from average
    negative_peak: f32,
    /// Average (DC component)
    average: f32,
    /// DC RMS value
    dc_rms: f32,
    /// AC RMS value
    ac_rms: f32,
    /// Positive duty cycle
    duty_cycle_pos: f32,
    /// Negative duty cycle
    duty_cycle_neg: f32,
    /// Rise time measurement
    rise_time: f32,
    /// Fall time measurement
    fall_time: f32,
    /// Crest factor (peak/RMS)
    crest_factor: f32,
    /// Total Harmonic Distortion
    thd: f32,
    /// Measured frequency (Hz)
    measured_frequency: f32,
    /// Measured period (s)
    measured_period: f32,
    /// Form factor (RMS / |mean|)
    form_factor: f32,
    /// Overshoot percentage
    overshoot: f32,
    /// Undershoot percentage
    undershoot: f32,
    /// Maximum slew rate (units/s)
    slew_rate_val: f32,
    /// Signal-to-noise ratio (dB)
    snr: f32,
    /// SINAD (dB)
    sinad: f32,
    /// Effective number of bits
    enob: f32,
    /// Spurious-free dynamic range (dB)
    sfdr: f32,
    /// Sample buffer reference
    samples: &'a mut [f32], // Use slice instead of Vec for no_std
}

/// Comprehensive signal quality metrics.
///
/// This structure contains all computed signal characteristics useful
/// for power electronics analysis and system characterization. The fields
/// mirror measurements available on modern DSOs and power analysers.
#[derive(Debug, Clone, Copy)]
pub struct SignalMetrics {
    /// Peak-to-peak amplitude (max − min)
    pub peak_to_peak: f32,
    /// Maximum sample value
    pub max: f32,
    /// Minimum sample value
    pub min: f32,
    /// Maximum positive excursion from the average
    pub positive_peak: f32,
    /// Maximum negative excursion from the average (as a positive magnitude)
    pub negative_peak: f32,
    /// Average value (DC component)
    pub average: f32,
    /// DC RMS value (includes DC component)
    pub dc_rms: f32,
    /// AC RMS value (DC component removed)
    pub ac_rms: f32,
    /// Positive duty cycle (fraction of time above zero)
    pub duty_cycle_pos: f32,
    /// Negative duty cycle (fraction of time below zero)
    pub duty_cycle_neg: f32,
    /// Rise time (10% to 90% of amplitude) in seconds
    pub rise_time: f32,
    /// Fall time (90% to 10% of amplitude) in seconds
    pub fall_time: f32,
    /// Crest factor (peak amplitude / RMS)
    pub crest_factor: f32,
    /// Total Harmonic Distortion percentage
    pub thd: f32,
    /// Measured frequency via zero-crossing detection (Hz)
    pub frequency: f32,
    /// Measured period (1/frequency) in seconds
    pub period: f32,
    /// Form factor: RMS / |mean| (for AC signals)
    pub form_factor: f32,
    /// Percentage overshoot above steady-state high level
    pub overshoot: f32,
    /// Percentage undershoot below steady-state low level
    pub undershoot: f32,
    /// Maximum slew rate |dV/dt| (units per second)
    pub slew_rate: f32,
    /// Signal-to-noise ratio in dB
    pub snr: f32,
    /// Signal-to-noise-and-distortion ratio in dB
    pub sinad: f32,
    /// Effective number of bits
    pub enob: f32,
    /// Spurious-free dynamic range in dB
    pub sfdr: f32,
}

impl<'a> Signal<'a> {
    /// Creates a new signal generator with specified parameters.
    ///
    /// The signal is immediately generated based on the provided parameters
    /// and stored in the provided buffer.
    ///
    /// # Parameters
    ///
    /// * `wave_type` - Type of waveform to generate
    /// * `amplitude` - Peak amplitude of the signal
    /// * `frequency` - Fundamental frequency in Hz
    /// * `samples_buffer` - Mutable slice to store generated samples
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::signal::signal::{Signal, SignalType};
    ///
    /// let mut buffer = [0.0; 1000];
    /// let signal = Signal::new(SignalType::Sine, 10.0, 50.0, &mut buffer);
    ///
    /// assert_eq!(signal.get_samples().len(), 1000);
    /// ```
    pub fn new(
        wave_type: SignalType,
        amplitude: f32,
        frequency: f32,
        samples_buffer: &'a mut [f32],
    ) -> Signal<'a> {
        let num_samples = samples_buffer.len() as u32;
        let is_dc = wave_type == SignalType::DC;
        let mut signal = Signal {
            wave_type,
            amplitude,
            phase: 0.0,
            frequency,
            num_samples,
            duty_cycle: 0.5, // Default 50% duty cycle
            voltage: amplitude,
            current: 0.0,
            port_frequency: if is_dc { None } else { Some(frequency) },
            phase_angle: None,
            active_power: 0.0,
            reactive_power: None,
            apparent_power: None,
            power_factor: None,
            displacement_pf: None,
            energy_imported: 0.0,
            energy_exported: 0.0,
            voltage_thd: None,
            current_thd: None,
            voltage_crest_factor: None,
            current_crest_factor: None,
            voltage_harmonics: [None; 16],
            current_harmonics: [None; 16],
            efficiency: None,
            losses: None,
            voltage_min: 0.0,
            voltage_max: 0.0,
            current_max: 0.0,
            power_max: 0.0,
            is_enabled: true,
            is_faulted: false,
            temperature: None,
            peak_to_peak: 0.0,
            max: 0.0,
            min: 0.0,
            positive_peak: 0.0,
            negative_peak: 0.0,
            average: 0.0,
            dc_rms: 0.0,
            ac_rms: 0.0,
            duty_cycle_pos: 0.0,
            duty_cycle_neg: 0.0,
            rise_time: 0.0,
            fall_time: 0.0,
            crest_factor: 0.0,
            thd: 0.0,
            measured_frequency: 0.0,
            measured_period: 0.0,
            form_factor: 0.0,
            overshoot: 0.0,
            undershoot: 0.0,
            slew_rate_val: 0.0,
            snr: 0.0,
            sinad: 0.0,
            enob: 0.0,
            sfdr: 0.0,
            samples: samples_buffer,
        };

        signal.generate_samples();
        signal.calculate_metrics();
        signal.voltage = if is_dc { signal.average } else { signal.dc_rms };
        signal.voltage_crest_factor = Some(signal.crest_factor);
        signal.voltage_thd = Some(signal.thd);
        signal
    }

    /// Creates a signal configured as an electrical power port.
    ///
    /// This constructor is intended for UPS interface points such as mains,
    /// battery, solar, load, and inverter output.
    pub fn new_power_port(
        voltage: f32,
        current: f32,
        frequency: Option<f32>,
        samples_buffer: &'a mut [f32],
    ) -> Signal<'a> {
        let mut signal = Signal::new(SignalType::DC, voltage, 0.0, samples_buffer);
        signal.voltage = voltage;
        signal.current = current;
        signal.port_frequency = frequency;
        signal.phase_angle = Some(0.0);
        signal.active_power = signal.instantaneous_power();
        signal.apparent_power = Some(libm::fabsf(signal.instantaneous_power()));
        signal.power_factor = Some(1.0);
        signal.displacement_pf = Some(1.0);
        signal
    }

    /// Sets the phase offset for the signal.
    ///
    /// # Parameters
    ///
    /// * `phase` - Phase offset in degrees
    ///
    /// After setting the phase, the signal is regenerated and metrics recalculated.
    pub fn set_phase(&mut self, phase: f32) {
        self.phase = phase;
        self.generate_samples();
        self.calculate_metrics();
        self.voltage = if self.wave_type == SignalType::DC {
            self.average
        } else {
            self.dc_rms
        };
        self.voltage_crest_factor = Some(self.crest_factor);
        self.voltage_thd = Some(self.thd);
    }

    /// Sets the duty cycle for PWM signals.
    ///
    /// # Parameters
    ///
    /// * `duty_cycle` - Duty cycle as a fraction (0.0 to 1.0)
    ///
    /// This parameter only affects PWM signals. After setting the duty cycle,
    /// the signal is regenerated and metrics recalculated.
    pub fn set_duty_cycle(&mut self, duty_cycle: f32) {
        self.duty_cycle = duty_cycle.max(0.0).min(1.0); // Clamp to [0,1]
        self.generate_samples();
        self.calculate_metrics();
        self.voltage = if self.wave_type == SignalType::DC {
            self.average
        } else {
            self.dc_rms
        };
        self.voltage_crest_factor = Some(self.crest_factor);
        self.voltage_thd = Some(self.thd);
    }

    /// Gets a reference to the generated sample buffer.
    ///
    /// # Returns
    ///
    /// A slice containing all generated signal samples.
    pub fn get_samples(&self) -> &[f32] {
        self.samples
    }

    /// Gets the computed signal metrics.
    ///
    /// # Returns
    ///
    /// A `SignalMetrics` struct containing all computed signal characteristics.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::signal::signal::{Signal, SignalType};
    ///
    /// let mut buffer = [0.0; 1000];
    /// let signal = Signal::new(SignalType::DC, 5.0, 0.0, &mut buffer);
    /// let metrics = signal.get_metrics();
    ///
    /// assert_eq!(metrics.max, 5.0);
    /// assert_eq!(metrics.min, 5.0);
    /// assert_eq!(metrics.average, 5.0);
    /// assert_eq!(metrics.dc_rms, 5.0);
    /// ```
    pub fn get_metrics(&self) -> SignalMetrics {
        SignalMetrics {
            peak_to_peak: self.peak_to_peak,
            max: self.max,
            min: self.min,
            positive_peak: self.positive_peak,
            negative_peak: self.negative_peak,
            average: self.average,
            dc_rms: self.dc_rms,
            ac_rms: self.ac_rms,
            duty_cycle_pos: self.duty_cycle_pos,
            duty_cycle_neg: self.duty_cycle_neg,
            rise_time: self.rise_time,
            fall_time: self.fall_time,
            crest_factor: self.crest_factor,
            thd: self.thd,
            frequency: self.measured_frequency,
            period: self.measured_period,
            form_factor: self.form_factor,
            overshoot: self.overshoot,
            undershoot: self.undershoot,
            slew_rate: self.slew_rate_val,
            snr: self.snr,
            sinad: self.sinad,
            enob: self.enob,
            sfdr: self.sfdr,
        }
    }

    /// Resets all waveform samples and calculated metrics to zero.
    pub fn reset_waveform(&mut self) {
        for sample in self.samples.iter_mut() {
            *sample = 0.0;
        }

        self.peak_to_peak = 0.0;
        self.max = 0.0;
        self.min = 0.0;
        self.positive_peak = 0.0;
        self.negative_peak = 0.0;
        self.average = 0.0;
        self.dc_rms = 0.0;
        self.ac_rms = 0.0;
        self.duty_cycle_pos = 0.0;
        self.duty_cycle_neg = 0.0;
        self.rise_time = 0.0;
        self.fall_time = 0.0;
        self.crest_factor = 0.0;
        self.thd = 0.0;
        self.measured_frequency = 0.0;
        self.measured_period = 0.0;
        self.form_factor = 0.0;
        self.overshoot = 0.0;
        self.undershoot = 0.0;
        self.slew_rate_val = 0.0;
        self.snr = 0.0;
        self.sinad = 0.0;
        self.enob = 0.0;
        self.sfdr = 0.0;
        self.voltage = 0.0;
        self.voltage_crest_factor = Some(0.0);
        self.voltage_thd = Some(0.0);
    }

    /// Resets all electrical interface parameters to defaults.
    pub fn reset_power_params(&mut self) {
        self.voltage = 0.0;
        self.current = 0.0;
        self.port_frequency = None;
        self.phase_angle = None;
        self.active_power = 0.0;
        self.reactive_power = None;
        self.apparent_power = None;
        self.power_factor = None;
        self.displacement_pf = None;
        self.energy_imported = 0.0;
        self.energy_exported = 0.0;
        self.voltage_thd = None;
        self.current_thd = None;
        self.voltage_crest_factor = None;
        self.current_crest_factor = None;
        self.voltage_harmonics = [None; 16];
        self.current_harmonics = [None; 16];
        self.efficiency = None;
        self.losses = None;
        self.voltage_min = 0.0;
        self.voltage_max = 0.0;
        self.current_max = 0.0;
        self.power_max = 0.0;
        self.is_enabled = true;
        self.is_faulted = false;
        self.temperature = None;
    }

    /// Returns the interface voltage in volts.
    pub fn voltage(&self) -> f32 {
        self.voltage
    }

    /// Sets the interface voltage in volts.
    pub fn set_voltage(&mut self, voltage: f32) {
        self.voltage = voltage;
    }

    /// Returns the interface current in amps.
    pub fn current(&self) -> f32 {
        self.current
    }

    /// Sets the interface current in amps.
    pub fn set_current(&mut self, current: f32) {
        self.current = current;
    }

    /// Returns the interface frequency in hertz (`None` for DC).
    pub fn frequency(&self) -> Option<f32> {
        self.port_frequency
    }

    /// Sets the interface frequency in hertz (`None` for DC).
    pub fn set_frequency(&mut self, frequency: Option<f32>) {
        self.port_frequency = frequency;
    }

    /// Returns the interface phase angle in degrees.
    pub fn phase_angle(&self) -> Option<f32> {
        self.phase_angle
    }

    /// Sets the interface phase angle in degrees.
    pub fn set_phase_angle(&mut self, phase_angle: Option<f32>) {
        self.phase_angle = phase_angle;
    }

    /// Returns the active power in watts.
    pub fn active_power(&self) -> f32 {
        self.active_power
    }

    /// Sets the active power in watts.
    pub fn set_active_power(&mut self, active_power: f32) {
        self.active_power = active_power;
    }

    /// Returns reactive power in vars.
    pub fn reactive_power(&self) -> Option<f32> {
        self.reactive_power
    }

    /// Sets reactive power in vars.
    pub fn set_reactive_power(&mut self, reactive_power: Option<f32>) {
        self.reactive_power = reactive_power;
    }

    /// Returns apparent power in volt-amps.
    pub fn apparent_power(&self) -> Option<f32> {
        self.apparent_power
    }

    /// Sets apparent power in volt-amps.
    pub fn set_apparent_power(&mut self, apparent_power: Option<f32>) {
        self.apparent_power = apparent_power;
    }

    /// Returns true power factor.
    pub fn power_factor(&self) -> Option<f32> {
        self.power_factor
    }

    /// Sets true power factor.
    pub fn set_power_factor(&mut self, power_factor: Option<f32>) {
        self.power_factor = power_factor;
    }

    /// Returns displacement power factor.
    pub fn displacement_pf(&self) -> Option<f32> {
        self.displacement_pf
    }

    /// Sets displacement power factor.
    pub fn set_displacement_pf(&mut self, displacement_pf: Option<f32>) {
        self.displacement_pf = displacement_pf;
    }

    /// Returns imported energy in watt-hours.
    pub fn energy_imported(&self) -> f64 {
        self.energy_imported
    }

    /// Sets imported energy in watt-hours.
    pub fn set_energy_imported(&mut self, energy_imported: f64) {
        self.energy_imported = energy_imported;
    }

    /// Returns exported energy in watt-hours.
    pub fn energy_exported(&self) -> f64 {
        self.energy_exported
    }

    /// Sets exported energy in watt-hours.
    pub fn set_energy_exported(&mut self, energy_exported: f64) {
        self.energy_exported = energy_exported;
    }

    /// Returns voltage THD in percent.
    pub fn voltage_thd(&self) -> Option<f32> {
        self.voltage_thd
    }

    /// Sets voltage THD in percent.
    pub fn set_voltage_thd(&mut self, voltage_thd: Option<f32>) {
        self.voltage_thd = voltage_thd;
    }

    /// Returns current THD in percent.
    pub fn current_thd(&self) -> Option<f32> {
        self.current_thd
    }

    /// Sets current THD in percent.
    pub fn set_current_thd(&mut self, current_thd: Option<f32>) {
        self.current_thd = current_thd;
    }

    /// Returns voltage crest factor.
    pub fn voltage_crest_factor(&self) -> Option<f32> {
        self.voltage_crest_factor
    }

    /// Sets voltage crest factor.
    pub fn set_voltage_crest_factor(&mut self, voltage_crest_factor: Option<f32>) {
        self.voltage_crest_factor = voltage_crest_factor;
    }

    /// Returns current crest factor.
    pub fn current_crest_factor(&self) -> Option<f32> {
        self.current_crest_factor
    }

    /// Sets current crest factor.
    pub fn set_current_crest_factor(&mut self, current_crest_factor: Option<f32>) {
        self.current_crest_factor = current_crest_factor;
    }

    /// Returns a voltage harmonic magnitude by harmonic order (`n = 1,2,3...`).
    pub fn voltage_harmonic(&self, n: u8) -> Option<f32> {
        self.voltage_harmonics
            .get(n as usize)
            .copied()
            .unwrap_or(None)
    }

    /// Sets a voltage harmonic magnitude by harmonic order (`n = 1,2,3...`).
    pub fn set_voltage_harmonic(&mut self, n: u8, value: Option<f32>) {
        if let Some(slot) = self.voltage_harmonics.get_mut(n as usize) {
            *slot = value;
        }
    }

    /// Returns a current harmonic magnitude by harmonic order (`n = 1,2,3...`).
    pub fn current_harmonic(&self, n: u8) -> Option<f32> {
        self.current_harmonics
            .get(n as usize)
            .copied()
            .unwrap_or(None)
    }

    /// Sets a current harmonic magnitude by harmonic order (`n = 1,2,3...`).
    pub fn set_current_harmonic(&mut self, n: u8, value: Option<f32>) {
        if let Some(slot) = self.current_harmonics.get_mut(n as usize) {
            *slot = value;
        }
    }

    /// Returns conversion efficiency in percent.
    pub fn efficiency(&self) -> Option<f32> {
        self.efficiency
    }

    /// Sets conversion efficiency in percent.
    pub fn set_efficiency(&mut self, efficiency: Option<f32>) {
        self.efficiency = efficiency;
    }

    /// Returns estimated losses in watts.
    pub fn losses(&self) -> Option<f32> {
        self.losses
    }

    /// Sets estimated losses in watts.
    pub fn set_losses(&mut self, losses: Option<f32>) {
        self.losses = losses;
    }

    /// Returns minimum supported voltage in volts.
    pub fn voltage_min(&self) -> f32 {
        self.voltage_min
    }

    /// Returns maximum supported voltage in volts.
    pub fn voltage_max(&self) -> f32 {
        self.voltage_max
    }

    /// Returns maximum supported current in amps.
    pub fn current_max(&self) -> f32 {
        self.current_max
    }

    /// Returns maximum supported power in watts.
    pub fn power_max(&self) -> f32 {
        self.power_max
    }

    /// Sets supported voltage/current/power limits.
    pub fn set_limits(
        &mut self,
        voltage_min: f32,
        voltage_max: f32,
        current_max: f32,
        power_max: f32,
    ) {
        self.voltage_min = voltage_min;
        self.voltage_max = voltage_max;
        self.current_max = current_max;
        self.power_max = power_max;
    }

    /// Returns whether the interface is enabled.
    pub fn is_enabled(&self) -> bool {
        self.is_enabled
    }

    /// Returns whether the interface is faulted.
    pub fn is_faulted(&self) -> bool {
        self.is_faulted
    }

    /// Sets interface enable and fault state.
    pub fn set_state(&mut self, is_enabled: bool, is_faulted: bool) {
        self.is_enabled = is_enabled;
        self.is_faulted = is_faulted;
    }

    /// Returns interface temperature in degrees Celsius.
    pub fn temperature(&self) -> Option<f32> {
        self.temperature
    }

    /// Sets interface temperature in degrees Celsius.
    pub fn set_temperature(&mut self, temperature: Option<f32>) {
        self.temperature = temperature;
    }

    /// Returns instantaneous power estimate in watts.
    pub fn instantaneous_power(&self) -> f32 {
        self.voltage * self.current
    }

    /// Generates signal samples based on the current parameters.
    ///
    /// This method fills the sample buffer with the appropriate waveform
    /// based on the signal type, amplitude, frequency, and other parameters.
    fn generate_samples(&mut self) {
        let sample_rate = self.num_samples as f32;

        for i in 0..self.num_samples {
            let t = i as f32 / sample_rate;
            let angular_freq = 2.0 * PI * self.frequency;
            let phase_rad = self.phase * PI / 180.0;

            let sample = match self.wave_type {
                SignalType::DC => self.amplitude,

                SignalType::Sine => self.amplitude * libm::sinf(angular_freq * t + phase_rad),

                SignalType::FullWaveRectifiedSine => {
                    self.amplitude * libm::fabsf(libm::sinf(angular_freq * t + phase_rad))
                }

                SignalType::HalfWaveRectifiedSine => {
                    let val = self.amplitude * libm::sinf(angular_freq * t + phase_rad);
                    if val > 0.0 { val } else { 0.0 }
                }

                SignalType::PWM => {
                    // For PWM, we assume the buffer represents one or more complete periods
                    // If frequency is high relative to num_samples, we just create a simple pattern
                    let samples_per_period = if self.frequency >= self.num_samples as f32 {
                        // High frequency case: treat the whole buffer as one period
                        self.num_samples as f32
                    } else {
                        // Normal case: calculate samples per period
                        self.num_samples as f32 / self.frequency
                    };

                    let sample_in_period = (i as f32) % samples_per_period;
                    let normalized_position = sample_in_period / samples_per_period;

                    if normalized_position < self.duty_cycle {
                        self.amplitude
                    } else {
                        0.0
                    }
                }

                SignalType::Triangular => {
                    let period = 1.0 / self.frequency;
                    let t_mod = (t + phase_rad / angular_freq) % period;
                    let normalized = t_mod / period;

                    if normalized < 0.5 {
                        self.amplitude * (4.0 * normalized - 1.0)
                    } else {
                        self.amplitude * (3.0 - 4.0 * normalized)
                    }
                }

                SignalType::Square => {
                    if libm::sinf(angular_freq * t + phase_rad) >= 0.0 {
                        self.amplitude
                    } else {
                        -self.amplitude
                    }
                }

                SignalType::Sawtooth => {
                    let period = 1.0 / self.frequency;
                    let t_mod = (t + phase_rad / angular_freq) % period;
                    self.amplitude * (2.0 * t_mod / period - 1.0)
                }
            };

            self.samples[i as usize] = sample;
        }
    }

    /// Calculates comprehensive signal metrics from the generated samples.
    ///
    /// This method computes all signal quality metrics including statistical
    /// measures, duty cycles, timing parameters, spectral analysis, and
    /// DSO/power-analyser measurements (frequency, form factor, slew rate,
    /// overshoot, undershoot, SNR, SINAD, ENOB, SFDR, THD).
    fn calculate_metrics(&mut self) {
        let n = self.num_samples as usize;
        if n == 0 {
            return;
        }

        // ----- 1. Basic statistics -----
        self.max = f32::NEG_INFINITY;
        self.min = f32::INFINITY;
        let mut sum: f64 = 0.0;
        let mut squared_sum: f64 = 0.0;
        let mut positive_samples: u32 = 0;

        for &sample in self.samples.iter() {
            self.max = libm::fmaxf(self.max, sample);
            self.min = libm::fminf(self.min, sample);
            sum += sample as f64;
            squared_sum += (sample as f64) * (sample as f64);
            if sample > 0.0 {
                positive_samples += 1;
            }
        }

        self.peak_to_peak = self.max - self.min;
        self.average = (sum / n as f64) as f32;
        self.positive_peak = self.max - self.average;
        self.negative_peak = self.average - self.min;

        // ----- 2. RMS calculations -----
        self.dc_rms = libm::sqrtf((squared_sum / n as f64) as f32);

        let mut ac_squared_sum: f64 = 0.0;
        for &sample in self.samples.iter() {
            let ac = sample - self.average;
            ac_squared_sum += (ac as f64) * (ac as f64);
        }
        self.ac_rms = libm::sqrtf((ac_squared_sum / n as f64) as f32);

        // ----- 3. Form factor: RMS / rectified-mean -----
        // For AC signals, the standard definition is RMS / mean_of_|x|.
        // For DC signals, it simplifies to 1.0.
        let mut rect_sum: f64 = 0.0;
        for &sample in self.samples.iter() {
            rect_sum += libm::fabsf(sample) as f64;
        }
        let rect_avg = (rect_sum / n as f64) as f32;
        self.form_factor = if rect_avg > 1e-12 {
            self.dc_rms / rect_avg
        } else {
            0.0
        };

        // ----- 4. Duty cycle -----
        if self.wave_type == SignalType::PWM {
            let mut high_samples: u32 = 0;
            for &sample in self.samples.iter() {
                if (sample - self.amplitude).abs() < 1e-6 {
                    high_samples += 1;
                }
            }
            self.duty_cycle_pos = high_samples as f32 / n as f32;
            self.duty_cycle_neg = 1.0 - self.duty_cycle_pos;
        } else {
            self.duty_cycle_pos = positive_samples as f32 / n as f32;
            self.duty_cycle_neg = 1.0 - self.duty_cycle_pos;
        }

        // ----- 5. Rise / fall time (10%–90%) -----
        let threshold_low = self.min + 0.1 * self.peak_to_peak;
        let threshold_high = self.min + 0.9 * self.peak_to_peak;
        let mut rise_samples: u32 = 0;
        let mut fall_samples: u32 = 0;
        let mut rising = false;
        let mut falling = false;

        for i in 1..n {
            let prev = self.samples[i - 1];
            let curr = self.samples[i];

            if prev <= threshold_low && curr > threshold_low {
                rising = true;
                falling = false;
                rise_samples = 0;
            }
            if rising && curr <= threshold_high {
                rise_samples += 1;
            }
            if prev >= threshold_high && curr < threshold_high {
                falling = true;
                rising = false;
                fall_samples = 0;
            }
            if falling && curr >= threshold_low {
                fall_samples += 1;
            }
        }

        // The sample buffer represents one period of the signal (sample_rate == num_samples)
        let sample_period = if self.frequency > 0.0 {
            1.0 / (self.frequency * n as f32)
        } else {
            1.0 / n as f32
        };
        self.rise_time = rise_samples as f32 * sample_period;
        self.fall_time = fall_samples as f32 * sample_period;

        // ----- 6. Crest factor -----
        if self.dc_rms > 1e-12 {
            let peak = libm::fmaxf(libm::fabsf(self.max), libm::fabsf(self.min));
            self.crest_factor = peak / self.dc_rms;
        } else {
            self.crest_factor = 0.0;
        }

        // ----- 7. Frequency measurement via zero-crossing -----
        // Buffer represents 1 second of data (sample_rate == num_samples)
        let sample_rate = n as f32;
        self.measured_frequency = measure_frequency(self.samples, sample_rate);
        self.measured_period = if self.measured_frequency > 0.0 {
            1.0 / self.measured_frequency
        } else {
            0.0
        };

        // ----- 8. Slew rate (max |dV/dt|) -----
        self.slew_rate_val = slew_rate(self.samples, sample_rate);

        // ----- 9. Overshoot / undershoot -----
        // Defined relative to the amplitude of the signal
        if self.peak_to_peak > 1e-12 {
            let expected_high = self.amplitude;
            let expected_low = -self.amplitude;

            // Overshoot: how far max exceeds expected high level
            let os = if self.max > expected_high {
                (self.max - expected_high) / self.peak_to_peak * 100.0
            } else {
                0.0
            };
            // Undershoot: how far min falls below expected low level
            let us = if self.min < expected_low {
                (expected_low - self.min) / self.peak_to_peak * 100.0
            } else {
                0.0
            };
            self.overshoot = os;
            self.undershoot = us;
        } else {
            self.overshoot = 0.0;
            self.undershoot = 0.0;
        }

        // ----- 10. DFT-based spectral analysis -----
        // Compute magnitude spectrum for THD, SNR, SINAD, ENOB, SFDR
        let half = n / 2;
        if half > 1 && self.frequency > 0.0 {
            // Find fundamental bin: bin = freq * N / sample_rate
            // Since sample_rate == N and the buffer holds 1 second, bin ≈ frequency
            let fundamental_bin = libm::roundf(self.frequency * n as f32 / sample_rate) as usize;

            if fundamental_bin > 0 && fundamental_bin < half {
                // Compute only the bins we need for THD (harmonics 1..max_harmonic)
                let max_harmonic_order = 16.min(half / fundamental_bin);
                let mut harmonic_mags = [0.0f32; 17]; // indices 1..=16

                for h in 1..=max_harmonic_order {
                    let bin = fundamental_bin * h;
                    if bin < half {
                        harmonic_mags[h] = self.compute_dft_bin_magnitude(bin);
                    }
                }

                let fund_mag = harmonic_mags[1];
                if fund_mag > 1e-12 {
                    // THD = sqrt(sum(H2² + H3² + ...)) / H1 × 100%
                    let mut harm_sum_sq: f64 = 0.0;
                    for h in 2..=max_harmonic_order {
                        let m = harmonic_mags[h] as f64;
                        harm_sum_sq += m * m;
                    }
                    self.thd = (libm::sqrt(harm_sum_sq) as f32 / fund_mag) * 100.0;

                    // SNR: power of fundamental / total noise power
                    // Noise = everything except fundamental & harmonics
                    let mut total_power: f64 = 0.0;
                    let mut signal_power: f64 = (fund_mag as f64) * (fund_mag as f64);
                    let mut harmonic_power: f64 = 0.0;

                    // Compute total power from DFT (Parseval)
                    for bin_idx in 1..half {
                        let mag = self.compute_dft_bin_magnitude(bin_idx);
                        total_power += (mag as f64) * (mag as f64);
                    }

                    for h in 2..=max_harmonic_order {
                        harmonic_power += (harmonic_mags[h] as f64) * (harmonic_mags[h] as f64);
                    }

                    let noise_power = total_power - signal_power - harmonic_power;
                    let noise_power = if noise_power > 0.0 {
                        noise_power
                    } else {
                        1e-30
                    };

                    self.snr = (10.0 * libm::log10(signal_power / noise_power)) as f32;

                    // SINAD: signal / (noise + distortion)
                    let nd_power = noise_power + harmonic_power;
                    let nd_power = if nd_power > 0.0 { nd_power } else { 1e-30 };
                    self.sinad = (10.0 * libm::log10(signal_power / nd_power)) as f32;

                    // ENOB
                    self.enob = (self.sinad - 1.76) / 6.02;

                    // SFDR: fundamental / largest spurious component (dB)
                    let mut max_spur: f32 = 0.0;
                    for bin_idx in 1..half {
                        if bin_idx != fundamental_bin {
                            let mag = self.compute_dft_bin_magnitude(bin_idx);
                            if mag > max_spur {
                                max_spur = mag;
                            }
                        }
                    }
                    if max_spur > 1e-12 {
                        self.sfdr = 20.0 * libm::log10f(fund_mag / max_spur);
                    } else {
                        self.sfdr = 120.0; // No spurious content
                    }
                }
            }
        }
    }

    /// Compute the DFT magnitude of a single frequency bin.
    ///
    /// Uses the standard DFT correlation formula normalised to amplitude.
    fn compute_dft_bin_magnitude(&self, bin: usize) -> f32 {
        let n = self.samples.len();
        if n == 0 {
            return 0.0;
        }
        let mut real: f64 = 0.0;
        let mut imag: f64 = 0.0;
        let freq = 2.0 * core::f64::consts::PI * bin as f64 / n as f64;

        for (i, &sample) in self.samples.iter().enumerate() {
            let angle = freq * i as f64;
            real += sample as f64 * libm::cos(angle);
            imag -= sample as f64 * libm::sin(angle);
        }

        // Normalise: 2/N for single-sided spectrum
        let mag = libm::sqrt(real * real + imag * imag) * 2.0 / n as f64;
        mag as f32
    }
}

// ============================================================================
// Standalone (free) signal-analysis functions
// ============================================================================

/// Measure signal frequency via zero-crossing detection.
///
/// Counts rising zero-crossings (negative→positive transitions) and returns
/// the estimated frequency in Hz.
///
/// # Parameters
///
/// * `samples` – time-domain sample buffer
/// * `sample_rate` – number of samples per second (Hz)
///
/// # Returns
///
/// Estimated frequency in Hz, or `0.0` if fewer than 2 zero-crossings are found.
///
/// # Examples
///
/// ```rust
/// use libpower::signal::signal::{measure_frequency, Signal, SignalType};
///
/// let mut buf = [0.0f32; 1000];
/// let sig = Signal::new(SignalType::Sine, 1.0, 5.0, &mut buf);
/// let freq = measure_frequency(sig.get_samples(), 1000.0);
/// assert!((freq - 5.0).abs() < 0.5);
/// ```
pub fn measure_frequency(samples: &[f32], sample_rate: f32) -> f32 {
    let n = samples.len();
    if n < 2 {
        return 0.0;
    }

    let mut crossings: u32 = 0;
    let mut first_crossing: usize = 0;
    let mut last_crossing: usize = 0;

    // Use the signal mean as the zero reference to handle DC offsets
    let mut sum: f64 = 0.0;
    for &s in samples.iter() {
        sum += s as f64;
    }
    let mean = (sum / n as f64) as f32;

    for i in 1..n {
        let prev = samples[i - 1] - mean;
        let curr = samples[i] - mean;
        // Rising zero crossing
        if prev < 0.0 && curr >= 0.0 {
            if crossings == 0 {
                first_crossing = i;
            }
            last_crossing = i;
            crossings += 1;
        }
    }

    if crossings < 2 {
        return 0.0;
    }

    let cycles = (crossings - 1) as f32;
    let span_samples = (last_crossing - first_crossing) as f32;
    if span_samples <= 0.0 {
        return 0.0;
    }

    cycles * sample_rate / span_samples
}

/// Compute the DFT magnitude spectrum of a real-valued signal.
///
/// The output contains `N/2` bins (DC excluded; bins 1..N/2-1).
/// Each bin magnitude is normalised to signal amplitude.
///
/// # Parameters
///
/// * `samples` – input time-domain samples
/// * `output` – output slice; must have length ≥ `samples.len() / 2`
///
/// # Panics
///
/// Panics if `output` is shorter than `samples.len() / 2`.
pub fn dft_magnitude_spectrum(samples: &[f32], output: &mut [f32]) {
    let n = samples.len();
    let half = n / 2;
    assert!(output.len() >= half, "Output buffer too small for DFT");

    for k in 0..half {
        let mut real: f64 = 0.0;
        let mut imag: f64 = 0.0;
        let freq = 2.0 * core::f64::consts::PI * k as f64 / n as f64;

        for (i, &sample) in samples.iter().enumerate() {
            let angle = freq * i as f64;
            real += sample as f64 * libm::cos(angle);
            imag -= sample as f64 * libm::sin(angle);
        }

        let mag = libm::sqrt(real * real + imag * imag) * 2.0 / n as f64;
        output[k] = mag as f32;
    }
    // DC bin correction (no ×2 factor)
    if half > 0 {
        output[0] /= 2.0;
    }
}

/// Goertzel algorithm – compute the magnitude at a single target frequency.
///
/// More efficient than a full DFT when only a few frequencies are needed.
///
/// # Parameters
///
/// * `samples` – time-domain sample buffer
/// * `target_freq` – frequency of interest in Hz
/// * `sample_rate` – sample rate in Hz
///
/// # Returns
///
/// Magnitude at the target frequency, normalised to amplitude.
pub fn goertzel_magnitude(samples: &[f32], target_freq: f32, sample_rate: f32) -> f32 {
    let n = samples.len();
    if n == 0 {
        return 0.0;
    }

    let k = (target_freq * n as f32 / sample_rate) as f64;
    let w = 2.0 * core::f64::consts::PI * k / n as f64;
    let coeff = 2.0 * libm::cos(w);

    let mut s0: f64 = 0.0;
    let mut s1: f64 = 0.0;
    let mut s2: f64;

    for &sample in samples.iter() {
        s2 = s1;
        s1 = s0;
        s0 = sample as f64 + coeff * s1 - s2;
    }

    let power = s0 * s0 + s1 * s1 - coeff * s0 * s1;
    let mag = libm::sqrt(if power > 0.0 { power } else { 0.0 });
    (mag * 2.0 / n as f64) as f32
}

/// Compute THD from a pre-computed magnitude spectrum.
///
/// THD = √(Σ H_n²) / H_1 × 100%  for n = 2,3,...,max available harmonic.
///
/// # Parameters
///
/// * `spectrum` – magnitude spectrum (bins 0..N/2); bin 0 is DC
/// * `fundamental_bin` – index of the fundamental frequency bin
///
/// # Returns
///
/// THD as a percentage.
pub fn compute_thd_from_spectrum(spectrum: &[f32], fundamental_bin: usize) -> f32 {
    if fundamental_bin == 0 || fundamental_bin >= spectrum.len() {
        return 0.0;
    }
    let fund = spectrum[fundamental_bin];
    if fund < 1e-12 {
        return 0.0;
    }

    let mut harm_sum_sq: f64 = 0.0;
    let mut h = 2;
    loop {
        let bin = fundamental_bin * h;
        if bin >= spectrum.len() {
            break;
        }
        let m = spectrum[bin] as f64;
        harm_sum_sq += m * m;
        h += 1;
    }

    (libm::sqrt(harm_sum_sq) as f32 / fund) * 100.0
}

/// Compute SNR from a pre-computed magnitude spectrum (dB).
///
/// SNR = 10 × log₁₀(P_signal / P_noise)
///
/// Signal power is the fundamental bin; noise is everything else except
/// DC and harmonic bins.
pub fn compute_snr(spectrum: &[f32], fundamental_bin: usize) -> f32 {
    if fundamental_bin == 0 || fundamental_bin >= spectrum.len() {
        return 0.0;
    }
    let fund = spectrum[fundamental_bin] as f64;
    let signal_power = fund * fund;

    let mut total_power: f64 = 0.0;
    let mut harmonic_power: f64 = 0.0;
    for (_i, &mag) in spectrum.iter().enumerate().skip(1) {
        let m = mag as f64;
        total_power += m * m;
    }

    // Identify harmonic bins
    let mut h = 2;
    loop {
        let bin = fundamental_bin * h;
        if bin >= spectrum.len() {
            break;
        }
        let m = spectrum[bin] as f64;
        harmonic_power += m * m;
        h += 1;
    }

    let noise_power = total_power - signal_power - harmonic_power;
    let noise_power = if noise_power > 0.0 {
        noise_power
    } else {
        1e-30
    };
    (10.0 * libm::log10(signal_power / noise_power)) as f32
}

/// Compute SINAD from a pre-computed magnitude spectrum (dB).
///
/// SINAD = 10 × log₁₀(P_signal / (P_noise + P_distortion))
pub fn compute_sinad(spectrum: &[f32], fundamental_bin: usize) -> f32 {
    if fundamental_bin == 0 || fundamental_bin >= spectrum.len() {
        return 0.0;
    }
    let fund = spectrum[fundamental_bin] as f64;
    let signal_power = fund * fund;

    let mut total_power: f64 = 0.0;
    for (_i, &mag) in spectrum.iter().enumerate().skip(1) {
        let m = mag as f64;
        total_power += m * m;
    }

    let nd_power = total_power - signal_power;
    let nd_power = if nd_power > 0.0 { nd_power } else { 1e-30 };
    (10.0 * libm::log10(signal_power / nd_power)) as f32
}

/// Compute Effective Number of Bits from SINAD.
///
/// ENOB = (SINAD_dB − 1.76) / 6.02
pub fn compute_enob(sinad_db: f32) -> f32 {
    (sinad_db - 1.76) / 6.02
}

/// Compute Spurious-Free Dynamic Range from a magnitude spectrum (dB).
///
/// SFDR = 20 × log₁₀(fundamental / largest_spur)
pub fn compute_sfdr(spectrum: &[f32], fundamental_bin: usize) -> f32 {
    if fundamental_bin == 0 || fundamental_bin >= spectrum.len() {
        return 0.0;
    }
    let fund = spectrum[fundamental_bin];
    let mut max_spur: f32 = 0.0;

    for (i, &mag) in spectrum.iter().enumerate().skip(1) {
        if i != fundamental_bin && mag > max_spur {
            max_spur = mag;
        }
    }

    if max_spur > 1e-12 {
        20.0 * libm::log10f(fund / max_spur)
    } else {
        120.0
    }
}

/// Compute active, reactive, apparent power and power factor from aligned
/// voltage and current waveforms.
///
/// # Parameters
///
/// * `voltage` – voltage samples (one or more complete periods)
/// * `current` – current samples (same length as voltage)
/// * `sample_rate` – sample rate in Hz (unused internally but kept for API symmetry)
///
/// # Returns
///
/// `(P, Q, S, PF)`:
/// - `P` – active (real) power in watts
/// - `Q` – reactive power in vars
/// - `S` – apparent power in VA
/// - `PF` – power factor (P / S)
///
/// # Panics
///
/// Panics if `voltage` and `current` have different lengths.
pub fn compute_power(voltage: &[f32], current: &[f32], _sample_rate: f32) -> (f32, f32, f32, f32) {
    assert_eq!(
        voltage.len(),
        current.len(),
        "V and I must have equal length"
    );
    let n = voltage.len();
    if n == 0 {
        return (0.0, 0.0, 0.0, 0.0);
    }

    let mut p_sum: f64 = 0.0;
    let mut v_sq_sum: f64 = 0.0;
    let mut i_sq_sum: f64 = 0.0;

    for idx in 0..n {
        let v = voltage[idx] as f64;
        let i = current[idx] as f64;
        p_sum += v * i;
        v_sq_sum += v * v;
        i_sq_sum += i * i;
    }

    let p = (p_sum / n as f64) as f32; // Active power
    let v_rms = libm::sqrt(v_sq_sum / n as f64) as f32;
    let i_rms = libm::sqrt(i_sq_sum / n as f64) as f32;
    let s = v_rms * i_rms; // Apparent power

    let pf = if s > 1e-12 { p / s } else { 0.0 };

    // Q = sqrt(S² - P²)
    let q_sq = (s as f64) * (s as f64) - (p as f64) * (p as f64);
    let q = libm::sqrt(if q_sq > 0.0 { q_sq } else { 0.0 }) as f32;

    (p, q, s, pf)
}

/// Compute phase difference between two waveforms using cross-correlation
/// at the fundamental frequency.
///
/// # Returns
///
/// Phase difference in degrees (signal B leads signal A → positive).
pub fn phase_difference(sig_a: &[f32], sig_b: &[f32], sample_rate: f32) -> f32 {
    let n = sig_a.len().min(sig_b.len());
    if n < 2 {
        return 0.0;
    }

    // Find fundamental of signal A via zero-crossing
    let freq = measure_frequency(sig_a, sample_rate);
    if freq <= 0.0 {
        return 0.0;
    }

    // DFT at the fundamental frequency for both signals
    let k = (freq * n as f32 / sample_rate) as f64;
    let w = 2.0 * core::f64::consts::PI * k / n as f64;

    let mut ra: f64 = 0.0;
    let mut ia: f64 = 0.0;
    let mut rb: f64 = 0.0;
    let mut ib: f64 = 0.0;

    for i in 0..n {
        let angle = w * i as f64;
        let cos_a = libm::cos(angle);
        let sin_a = libm::sin(angle);
        ra += sig_a[i] as f64 * cos_a;
        ia -= sig_a[i] as f64 * sin_a;
        rb += sig_b[i] as f64 * cos_a;
        ib -= sig_b[i] as f64 * sin_a;
    }

    let phase_a = libm::atan2(ia, ra);
    let phase_b = libm::atan2(ib, rb);
    let mut diff = (phase_b - phase_a) * 180.0 / core::f64::consts::PI;

    // Normalise to [−180, 180]
    while diff > 180.0 {
        diff -= 360.0;
    }
    while diff < -180.0 {
        diff += 360.0;
    }

    diff as f32
}

/// Apply a window function to samples in-place.
///
/// # Parameters
///
/// * `samples` – mutable sample buffer
/// * `window` – window type to apply
///
/// # Examples
///
/// ```rust
/// use libpower::signal::signal::{apply_window, WindowType};
///
/// let mut buf = [1.0f32; 64];
/// apply_window(&mut buf, WindowType::Hanning);
/// // First and last samples should be near zero
/// assert!(buf[0].abs() < 0.01);
/// assert!(buf[63].abs() < 0.01);
/// ```
pub fn apply_window(samples: &mut [f32], window: WindowType) {
    let n = samples.len();
    if n <= 1 {
        return;
    }
    let nm1 = (n - 1) as f32;

    for i in 0..n {
        let w = match window {
            WindowType::Rectangular => 1.0,
            WindowType::Hanning => 0.5 * (1.0 - libm::cosf(2.0 * PI * i as f32 / nm1)),
            WindowType::Hamming => 0.54 - 0.46 * libm::cosf(2.0 * PI * i as f32 / nm1),
            WindowType::Blackman => {
                0.42 - 0.5 * libm::cosf(2.0 * PI * i as f32 / nm1)
                    + 0.08 * libm::cosf(4.0 * PI * i as f32 / nm1)
            }
            WindowType::FlatTop => {
                let a0 = 0.21557895;
                let a1 = 0.41663158;
                let a2 = 0.277263158;
                let a3 = 0.083578947;
                let a4 = 0.006947368;
                a0 - a1 * libm::cosf(2.0 * PI * i as f32 / nm1)
                    + a2 * libm::cosf(4.0 * PI * i as f32 / nm1)
                    - a3 * libm::cosf(6.0 * PI * i as f32 / nm1)
                    + a4 * libm::cosf(8.0 * PI * i as f32 / nm1)
            }
        };
        samples[i] *= w;
    }
}

/// Compute the maximum slew rate |dV/dt| of a sampled signal.
///
/// # Parameters
///
/// * `samples` – time-domain samples
/// * `sample_rate` – samples per second
///
/// # Returns
///
/// Maximum absolute rate-of-change in units per second.
pub fn slew_rate(samples: &[f32], sample_rate: f32) -> f32 {
    if samples.len() < 2 || sample_rate <= 0.0 {
        return 0.0;
    }

    let dt = 1.0 / sample_rate;
    let mut max_sr: f32 = 0.0;

    for i in 1..samples.len() {
        let dv = libm::fabsf(samples[i] - samples[i - 1]);
        let sr = dv / dt;
        if sr > max_sr {
            max_sr = sr;
        }
    }

    max_sr
}

/// Add a DC offset to every sample in-place.
pub fn add_dc_offset(samples: &mut [f32], offset: f32) {
    for s in samples.iter_mut() {
        *s += offset;
    }
}

/// Scale every sample by a constant factor in-place.
pub fn scale(samples: &mut [f32], factor: f32) {
    for s in samples.iter_mut() {
        *s *= factor;
    }
}

/// Element-wise addition of two signals into an output buffer.
///
/// `output[i] = a[i] + b[i]` for `i` in `0..min(a.len(), b.len(), output.len())`.
pub fn add_signals(a: &[f32], b: &[f32], output: &mut [f32]) {
    let n = a.len().min(b.len()).min(output.len());
    for i in 0..n {
        output[i] = a[i] + b[i];
    }
}
