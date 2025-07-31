//! # libpower - A no_std Rust library for power electronics, optimized for use on microcontrollers
//!
//! `libpower` is a comprehensive `no_std` Rust library designed for power electronics applications
//! in embedded systems. It provides a collection of digital signal processing (DSP) algorithms,
//! control systems, and power electronics primitives optimized for real-time control applications.
//!
//! ## Features
//!
//! - **No Standard Library Dependencies**: Built for embedded systems with `#![no_std]`
//! - **Real-time Performance**: Optimized for deterministic, low-latency control loops
//! - **Comprehensive Control Systems**: PID, PI, and advanced pole-zero controllers
//! - **Digital Signal Processing**: High-performance filters (Butterworth, Chebyshev)
//! - **Power Electronics Algorithms**: MPPT, PLL, coordinate transformations
//! - **Signal Generation**: Configurable waveform generators with metrics
//!
//! ## Architecture
//!
//! The library is organized into several modules, each targeting specific aspects of
//! power electronics control:
//!
//! ### Core Modules
//!
//! - [`battery`] - Battery management and State of Charge estimation
//! - [`control`] - Digital control systems (PID, PI, pole-zero controllers)
//! - [`filter`] - Digital signal processing filters
//! - [`mppt`] - Maximum Power Point Tracking algorithms
//! - [`pll`] - Phase-Locked Loop implementations
//! - [`signal`] - Signal generation and analysis tools
//! - [`transform`] - Coordinate transformations (Clarke, Park, etc.)
//!
//! ## Usage Patterns
//!
//! ### Basic Control Loop
//!
//! ```rust
//! use libpower::control::cntl_pi::ControllerPI;
//!
//! // Create a PI controller with gains
//! let mut controller = ControllerPI::with_gains(1.0, 0.1);
//! controller.set_limits(-10.0, 10.0);
//!
//! // In your control loop
//! let setpoint = 5.0;
//! let measurement = 4.5;
//! let output = controller.calculate(setpoint, measurement);
//! ```
//!
//! ### Digital Filtering
//!
//! ```rust
//! use libpower::filter::butterworth_lpf::ButterworthLPF;
//!
//! // Create and initialize a 4th-order Butterworth low-pass filter
//! let mut filter = ButterworthLPF::<4>::new_uninit();
//! filter.init(4, 1000.0, 44100.0); // 4th order, 1kHz cutoff, 44.1kHz sampling
//!
//! // Process samples
//! let input_sample = 1.0;
//! let filtered_output = filter.process(input_sample);
//! ```
//!
//! ### MPPT Implementation
//!
//! ```rust
//! use libpower::mppt::perturb_and_observe::MPPT;
//!
//! let mut mppt = MPPT::new();
//! mppt.set_step_size(0.1);
//! mppt.set_mppt_v_out_max(24.0);
//! mppt.set_mppt_v_out_min(12.0);
//!
//! // In your control loop
//! let pv_current = 5.0; // Example: 5A
//! let pv_voltage = 18.0; // Example: 18V
//! mppt.calculate(pv_current, pv_voltage);
//! let reference_voltage = mppt.get_mppt_v_out();
//! ```
//!
//! ### Battery State of Charge Estimation
//!
//! ```rust
//! use libpower::battery::soc::Battery;
//!
//! let mut soc_estimator = Battery::new(0.8, 50.0 * 3600.0, 1.0);
//! soc_estimator.set_process_noise(1e-5);
//! soc_estimator.set_measurement_noise(5e-4);
//!
//! // In your control loop
//! let battery_current = -15.0; // 15A discharge
//! let terminal_voltage = 3.6; // Measured terminal voltage
//! let estimated_soc = soc_estimator.update(battery_current, terminal_voltage);
//! ```
//!
//! ## Design Principles
//!
//! ### Performance
//! - All algorithms use single-precision floating-point arithmetic (`f32`)
//! - Minimal memory allocation (stack-based, pre-allocated buffers)
//! - Optimized for real-time control loops (typically 1-100 kHz)
//!
//! ### Safety
//! - Saturation limits and bounds checking where appropriate
//! - Graceful handling of edge cases and invalid inputs
//! - Clear error reporting through `Result` types where applicable
//!
//! ### Modularity
//! - Each algorithm is self-contained with minimal dependencies
//! - State variables are encapsulated within structs
//! - Configurable parameters with sensible defaults
//!
//! ## Target Applications
//!
//! - DC-DC converters and power supplies
//! - Motor drives and inverters
//! - Solar inverters and MPPT controllers
//! - Grid-tied power electronics
//! - Battery management systems
//! - Power factor correction circuits
//!
//! ## Examples
//!
//! See the individual module documentation and the `tests/` directory for comprehensive
//! examples of each algorithm and typical usage patterns.

#![no_std]

pub mod battery;
pub mod control;
pub mod filter;
pub mod modulation;
pub mod motor_control;
pub mod mppt;
pub mod pll;
pub mod signal;
pub mod transform;
