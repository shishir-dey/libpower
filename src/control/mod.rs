//! # Digital Control Systems
//!
//! This module provides a comprehensive collection of digital control algorithms commonly
//! used in power electronics and embedded control systems. All controllers are implemented
//! as discrete-time systems suitable for real-time control loops.
//!
//! ## Available Controllers
//!
//! ### Linear Controllers
//! - [`cntl_pi`] - Proportional-Integral (PI) controller with anti-windup
//! - [`cntl_pid`] - Proportional-Integral-Derivative (PID) controller
//!
//! ### Advanced Controllers
//! - [`cntl_2p2z`] - Two-pole, two-zero controller (2P2Z compensator)
//! - [`cntl_3p3z`] - Three-pole, three-zero controller (3P3Z compensator)
//!
//! ## Controller Selection Guide
//!
//! ### PI Controller
//! - **Best for**: DC-DC converters, current loops, simple voltage regulation
//! - **Advantages**: Zero steady-state error, built-in anti-windup
//! - **Limitations**: Cannot add zeros for improved transient response
//!
//! ### PID Controller
//! - **Best for**: General-purpose control, systems requiring derivative action
//! - **Advantages**: Includes derivative term for improved transient response
//! - **Limitations**: Sensitive to noise, requires tuning of three parameters
//!
//! ### 2P2Z Controller
//! - **Best for**: DC-DC converter voltage loops, single-phase power supplies
//! - **Advantages**: Two zeros for improved transient response and stability margins
//! - **Applications**: Buck/boost converters, battery chargers
//!
//! ### 3P3Z Controller
//! - **Best for**: Three-phase systems, complex power conversion topologies
//! - **Advantages**: Additional poles and zeros for precise frequency response shaping
//! - **Applications**: Grid-tied inverters, motor drives, advanced power supplies
//!
//! ## Design Considerations
//!
//! ### Sampling Frequency
//! All controllers assume a fixed sampling period. Typical sampling frequencies:
//! - Current loops: 10-100 kHz
//! - Voltage loops: 1-10 kHz
//! - Outer control loops: 100 Hz - 1 kHz
//!
//! ### Saturation and Anti-windup
//! All controllers include configurable output saturation limits to prevent:
//! - Integrator windup
//! - Actuator saturation
//! - System instability
//!
//! ### Coefficient Calculation
//! Controller coefficients are typically designed using:
//! - Bode plot analysis
//! - Root locus techniques
//! - Digital control design tools (MATLAB, Python control)
//!
//! ## Example Usage
//!
//! ```rust
//! use libpower::control::cntl_pi::ControllerPI;
//!
//! // Create PI controller for current loop
//! let mut current_controller = ControllerPI::with_gains(0.5, 100.0);
//! current_controller.set_limits(-1.0, 1.0); // ±1.0 duty cycle
//!
//! // Control loop execution
//! let current_reference = 5.0; // 5A reference
//! let current_measurement = 4.8; // 4.8A measured
//! let duty_cycle = current_controller.calculate(current_reference, current_measurement);
//! ```

pub mod cntl_2p2z;
pub mod cntl_3p3z;
pub mod cntl_pi;
pub mod cntl_pid;
