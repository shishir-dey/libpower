//! # Maximum Power Point Tracking (MPPT) Algorithms
//!
//! This module provides implementations of Maximum Power Point Tracking algorithms
//! commonly used in photovoltaic (PV) systems and other renewable energy applications.
//! MPPT algorithms optimize power extraction from variable sources like solar panels.
//!
//! ## Theory
//!
//! The maximum power point (MPP) of a photovoltaic array varies with:
//! - **Irradiance levels**: Higher light intensity increases power output
//! - **Temperature**: Higher temperatures generally reduce voltage and power
//! - **Partial shading**: Can create multiple local maxima
//! - **Array degradation**: Aging affects the characteristic curves
//!
//! ## Available Algorithms
//!
//! ### Perturb and Observe (P&O)
//! - [`perturb_and_observe`] - Classical hill-climbing algorithm
//! - **Principle**: Perturb voltage and observe power change
//! - **Advantages**: Simple implementation, low computational cost
//! - **Disadvantages**: Oscillates around MPP, can be confused by rapid irradiance changes
//!
//! ### Incremental Conductance (IC)
//! - [`incremental_conductance`] - Advanced gradient-based method
//! - **Principle**: Uses dP/dV = 0 condition at MPP
//! - **Advantages**: Can detect when MPP is reached, better dynamic response
//! - **Disadvantages**: More complex implementation, sensitive to noise
//!
//! ## Algorithm Comparison
//!
//! | Feature | P&O | IC |
//! |---------|-----|-----|
//! | Simplicity | High | Medium |
//! | Speed | Fast | Medium |
//! | Steady-state accuracy | Medium | High |
//! | Dynamic response | Medium | Good |
//! | Noise sensitivity | Low | Medium |
//! | Memory requirements | Low | Low |
//!
//! ## Implementation Considerations
//!
//! ### Step Size Selection
//! - **Large steps**: Faster tracking, more oscillation
//! - **Small steps**: Slower tracking, less oscillation
//! - **Adaptive step size**: Optimal compromise (not implemented)
//!
//! ### Sampling Frequency
//! - **Typical range**: 1-100 Hz depending on application
//! - **Trade-offs**: Faster sampling vs. computational load
//! - **Consideration**: PV array time constants (typically slow)
//!
//! ### Practical Considerations
//! - **Voltage and current measurement accuracy**
//! - **Converter dynamics and control loop interactions**
//! - **Protection against out-of-range operating points**
//! - **Startup and shutdown procedures**
//!
//! ## Applications
//!
//! - Grid-tied solar inverters
//! - Battery charging systems
//! - Standalone PV systems
//! - Solar water pumping
//! - Electric vehicle solar charging
//! - Building-integrated photovoltaics (BIPV)
//!
//! ## Example Usage
//!
//! ```rust
//! use libpower::mppt::perturb_and_observe::MPPT;
//!
//! // Create and configure MPPT tracker
//! let mut mppt = MPPT::new();
//! mppt.set_step_size(0.1);           // 0.1V step size
//! mppt.set_mppt_v_out_max(24.0);     // 24V maximum
//! mppt.set_mppt_v_out_min(12.0);     // 12V minimum
//!
//! // Simulate control loop samples
//! let pv_measurements = [
//!     (5.0, 18.0),  // (current, voltage) pairs
//!     (5.1, 18.2),
//!     (5.2, 18.1),
//! ];
//!
//! for (pv_current, pv_voltage) in pv_measurements.iter() {
//!     mppt.calculate(*pv_current, *pv_voltage);     // Run MPPT algorithm
//!     let reference_voltage = mppt.get_mppt_v_out(); // Get voltage reference
//!     // In real application: set_converter_reference(reference_voltage);
//! }
//! ```

pub mod incremental_conductance;
pub mod perturb_and_observe;
