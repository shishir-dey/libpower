//! # Digital Signal Processing Filters
//!
//! This module provides a collection of high-performance digital filters commonly used
//! in power electronics and embedded control systems. All filters are implemented as
//! second-order sections (SOS) for numerical stability and modularity.
//!
//! ## Available Filter Types
//!
//! ### Butterworth Filters
//! - [`butterworth_lpf`] - Butterworth low-pass filter (maximally flat passband)
//! - [`butterworth_hpf`] - Butterworth high-pass filter (maximally flat passband)
//!
//! ### Chebyshev Type I Filters  
//! - [`chebyshev_lpf`] - Chebyshev low-pass filter (equiripple passband)
//! - [`chebyshev_hpf`] - Chebyshev high-pass filter (equiripple passband)
//!
//! ## Filter Characteristics
//!
//! ### Butterworth Filters
//! - **Advantages**: Maximally flat passband, no ripple, good phase response
//! - **Applications**: Anti-aliasing, general-purpose filtering, measurement systems
//! - **Roll-off**: 20n dB/decade (where n is the filter order)
//!
//! ### Chebyshev Type I Filters
//! - **Advantages**: Steeper roll-off than Butterworth, compact implementation
//! - **Disadvantages**: Passband ripple, more phase distortion
//! - **Applications**: Anti-aliasing with tight transition bands, EMI filtering
//! - **Roll-off**: Steeper than Butterworth for same order
//!
//! ## Implementation Details
//!
//! ### Second-Order Sections (SOS)
//! All filters are implemented using cascaded second-order sections:
//! - **Numerical Stability**: Reduced coefficient sensitivity
//! - **Modularity**: Easy to implement different orders
//! - **Efficiency**: Optimized for real-time applications
//!
//! ### Supported Orders
//! - **Range**: 2nd to 8th order (even orders only)
//! - **Automatic Adjustment**: Invalid orders are bounded and rounded
//! - **Sections**: Each filter uses order/2 second-order sections
//!
//! ## Usage Guidelines
//!
//! ### Sampling Frequency Considerations
//! - Ensure fs > 2 * fc (Nyquist criterion)
//! - Typical ratio: fs/fc > 10 for good performance
//! - Account for transition band requirements
//!
//! ### Filter Selection
//! - **Butterworth**: Choose for phase-critical applications
//! - **Chebyshev**: Choose when sharp cutoff is required
//! - **Order Selection**: Higher order = sharper cutoff, more delay
//!
//! ## Example Usage
//!
//! ```rust
//! use libpower::filter::butterworth_lpf::ButterworthLPF;
//!
//! // Create and initialize a 4th-order Butterworth low-pass filter
//! let mut filter = ButterworthLPF::<4>::new_uninit();
//! filter.init(4, 1000.0, 44100.0); // 4th order, 1kHz cutoff, 44.1kHz sampling
//!
//! // Process signal samples
//! let input_signal = [1.0, 0.5, -0.5, -1.0, 0.0];
//! let mut filtered_signal = Vec::new();
//! for sample in input_signal.iter() {
//!     let filtered = filter.process(*sample);
//!     filtered_signal.push(filtered);
//! }
//! ```

pub mod butterworth_hpf;
pub mod butterworth_lpf;
pub mod chebyshev_hpf;
pub mod chebyshev_lpf;
