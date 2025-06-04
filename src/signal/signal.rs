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
    /// Phase offset in radians
    phase: f32,
    /// Fundamental frequency in Hz
    frequency: f32,
    /// Number of samples in the buffer
    num_samples: u32,
    /// Duty cycle for PWM signals (0.0 to 1.0)
    duty_cycle: f32,
    // Internal measurements
    /// Peak-to-peak amplitude
    peak_to_peak: f32,
    /// Maximum sample value
    max: f32,
    /// Minimum sample value
    min: f32,
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
    /// Sample buffer reference
    samples: &'a mut [f32], // Use slice instead of Vec for no_std
}

/// Comprehensive signal quality metrics.
///
/// This structure contains all computed signal characteristics useful
/// for power electronics analysis and system characterization.
#[derive(Debug, Clone, Copy)]
pub struct SignalMetrics {
    /// Peak-to-peak amplitude (max - min)
    pub peak_to_peak: f32,
    /// Maximum sample value
    pub max: f32,
    /// Minimum sample value
    pub min: f32,
    /// Average value (DC component)
    pub average: f32,
    /// DC RMS value
    pub dc_rms: f32,
    /// AC RMS value (excluding DC)
    pub ac_rms: f32,
    /// Positive duty cycle (fraction of time above zero)
    pub duty_cycle_pos: f32,
    /// Negative duty cycle (fraction of time below zero)
    pub duty_cycle_neg: f32,
    /// Rise time (10% to 90% of amplitude)
    pub rise_time: f32,
    /// Fall time (90% to 10% of amplitude)
    pub fall_time: f32,
    /// Crest factor (peak amplitude / RMS)
    pub crest_factor: f32,
    /// Total Harmonic Distortion percentage
    pub thd: f32,
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
        let mut signal = Signal {
            wave_type,
            amplitude,
            phase: 0.0,
            frequency,
            num_samples,
            duty_cycle: 0.5, // Default 50% duty cycle
            peak_to_peak: 0.0,
            max: 0.0,
            min: 0.0,
            average: 0.0,
            dc_rms: 0.0,
            ac_rms: 0.0,
            duty_cycle_pos: 0.0,
            duty_cycle_neg: 0.0,
            rise_time: 0.0,
            fall_time: 0.0,
            crest_factor: 0.0,
            thd: 0.0,
            samples: samples_buffer,
        };

        signal.generate_samples();
        signal.calculate_metrics();
        signal
    }

    /// Sets the phase offset for the signal.
    ///
    /// # Parameters
    ///
    /// * `phase` - Phase offset in radians
    ///
    /// After setting the phase, the signal is regenerated and metrics recalculated.
    pub fn set_phase(&mut self, phase: f32) {
        self.phase = phase;
        self.generate_samples();
        self.calculate_metrics();
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
            average: self.average,
            dc_rms: self.dc_rms,
            ac_rms: self.ac_rms,
            duty_cycle_pos: self.duty_cycle_pos,
            duty_cycle_neg: self.duty_cycle_neg,
            rise_time: self.rise_time,
            fall_time: self.fall_time,
            crest_factor: self.crest_factor,
            thd: self.thd,
        }
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
                    if val > 0.0 {
                        val
                    } else {
                        0.0
                    }
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
    /// measures, duty cycles, timing parameters, and harmonic content.
    fn calculate_metrics(&mut self) {
        // Basic statistics
        self.max = f32::NEG_INFINITY;
        self.min = f32::INFINITY;
        let mut sum = 0.0;
        let mut squared_sum = 0.0;
        let mut positive_samples = 0;

        for &sample in self.samples.iter() {
            self.max = libm::fmaxf(self.max, sample);
            self.min = libm::fminf(self.min, sample);
            sum += sample;
            squared_sum += sample * sample;
            if sample > 0.0 {
                positive_samples += 1;
            }
        }

        self.peak_to_peak = self.max - self.min;
        self.average = sum / self.num_samples as f32;

        // RMS calculations
        self.dc_rms = libm::sqrtf(squared_sum / self.num_samples as f32);

        // AC RMS (remove DC component)
        let mut ac_squared_sum = 0.0;
        for &sample in self.samples.iter() {
            let ac_sample = sample - self.average;
            ac_squared_sum += ac_sample * ac_sample;
        }
        self.ac_rms = libm::sqrtf(ac_squared_sum / self.num_samples as f32);

        // Duty cycle calculations
        if self.wave_type == SignalType::PWM {
            // For PWM, count samples at high level (amplitude)
            let mut high_samples = 0;
            for &sample in self.samples.iter() {
                if (sample - self.amplitude).abs() < 1e-6 {
                    high_samples += 1;
                }
            }
            self.duty_cycle_pos = high_samples as f32 / self.num_samples as f32;
            self.duty_cycle_neg = 1.0 - self.duty_cycle_pos;
        } else {
            // For other signals, count positive samples
            self.duty_cycle_pos = positive_samples as f32 / self.num_samples as f32;
            self.duty_cycle_neg = 1.0 - self.duty_cycle_pos;
        }

        // Rise and fall time calculations (10% to 90%)
        let threshold_low = self.min + 0.1 * self.peak_to_peak;
        let threshold_high = self.min + 0.9 * self.peak_to_peak;

        let mut rise_samples = 0;
        let mut fall_samples = 0;
        let mut rising = false;
        let mut falling = false;

        for i in 1..self.samples.len() {
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

        let sample_period = 1.0 / (self.frequency * self.num_samples as f32);
        self.rise_time = rise_samples as f32 * sample_period;
        self.fall_time = fall_samples as f32 * sample_period;

        // Crest factor: peak value divided by RMS value
        if self.dc_rms != 0.0 {
            let peak = libm::fmaxf(libm::fabsf(self.max), libm::fabsf(self.min));
            self.crest_factor = peak / self.dc_rms;
        }

        // Total Harmonic Distortion (simplified - only considers first 5 harmonics)
        if self.wave_type == SignalType::Sine {
            let fundamental = self.frequency;
            let mut harmonic_power = 0.0;

            for i in 2..=5 {
                let harmonic_freq = fundamental * i as f32;
                let harmonic_amplitude = self.compute_fft_magnitude(harmonic_freq);
                harmonic_power += harmonic_amplitude * harmonic_amplitude;
            }

            let fundamental_amplitude = self.compute_fft_magnitude(fundamental);
            if fundamental_amplitude != 0.0 {
                self.thd = (libm::sqrtf(harmonic_power) / fundamental_amplitude) * 100.0;
            }
        }
    }

    /// Computes simplified FFT magnitude for THD estimation.
    ///
    /// This is a simplified implementation for demonstration purposes.
    /// A full THD calculation would require a complete FFT implementation.
    ///
    /// # Parameters
    ///
    /// * `frequency` - Fundamental frequency for analysis
    ///
    /// # Returns
    ///
    /// Estimated THD percentage as a simplified metric.
    fn compute_fft_magnitude(&self, frequency: f32) -> f32 {
        let sample_rate = self.num_samples as f32;
        let mut real = 0.0;
        let mut imag = 0.0;

        for (i, &sample) in self.samples.iter().enumerate() {
            let t = i as f32 / sample_rate;
            let angle = 2.0 * PI * frequency * t;
            real += sample * libm::cosf(angle);
            imag += sample * libm::sinf(angle);
        }

        libm::sqrtf(real * real + imag * imag) / self.num_samples as f32
    }
}
