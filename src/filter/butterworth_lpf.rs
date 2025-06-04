//! # Butterworth Low-Pass Filter
//!
//! This module implements a digital Butterworth low-pass filter using cascaded
//! second-order sections (SOS). The Butterworth filter provides maximally flat
//! passband response with no ripple, making it ideal for applications requiring
//! smooth frequency response.
//!
//! ## Theory
//!
//! The Butterworth filter is characterized by:
//! - **Maximally flat passband**: No ripple in the passband
//! - **Monotonic stopband**: Smooth rolloff without oscillations  
//! - **Good phase response**: Minimal phase distortion in passband
//! - **Roll-off rate**: 20n dB/decade (where n is the filter order)
//!
//! ## Transfer Function
//!
//! Each second-order section implements:
//! ```text
//! H(z) = a₀(1 + 2z⁻¹ + z⁻²) / (1 + d₁z⁻¹ + d₂z⁻²)
//! ```
//!
//! Where coefficients are calculated using the bilinear transformation (Tustin method)
//! from the analog prototype.
//!
//! ## Applications
//!
//! - Anti-aliasing filters for ADC inputs
//! - Noise reduction in measurement systems
//! - Signal conditioning in control loops
//! - Audio and communications applications
//! - EMI filtering where phase response is critical

use core::cmp::{max, min};
use libm::{sinf, tanf};

/// Butterworth low-pass filter implemented using cascaded second-order sections.
///
/// This filter provides maximally flat passband response with configurable order
/// from 2 to 8. The implementation uses the Direct Form II structure for each
/// second-order section, providing good numerical properties and real-time performance.
///
/// # Type Parameters
///
/// * `N` - Maximum number of second-order sections (typically 4 for 8th-order filter)
///
/// # Examples
///
/// ## Basic Usage
///
/// ```rust
/// use libpower::filter::butterworth_lpf::ButterworthLPF;
///
/// // Create and initialize a 4th-order filter
/// let mut filter = ButterworthLPF::<4>::new_uninit();
/// filter.init(4, 1000.0, 44100.0); // 4th order, 1kHz cutoff, 44.1kHz sampling
///
/// assert_eq!(filter.get_order(), 4);
/// assert_eq!(filter.get_n(), 2); // 2 second-order sections
///
/// // Process a sample
/// let output = filter.process(1.0);
/// assert!(!output.is_nan());
/// ```
///
/// ## Different Filter Orders
///
/// ```rust
/// use libpower::filter::butterworth_lpf::ButterworthLPF;
///
/// // Test various orders (automatically bounded to even values 2-8)
/// for order in &[2, 4, 6, 8] {
///     let mut filter = ButterworthLPF::<4>::new_uninit();
///     filter.init(*order, 1000.0, 44100.0);
///     assert_eq!(filter.get_order(), *order);
///
///     // Test basic functionality
///     let output = filter.process(1.0);
///     assert!(!output.is_nan());
///     assert!(!output.is_infinite());
/// }
/// ```
///
/// ## Reset Filter State
///
/// ```rust
/// use libpower::filter::butterworth_lpf::ButterworthLPF;
///
/// let mut filter = ButterworthLPF::<4>::new_uninit();
/// filter.init(4, 1000.0, 44100.0);
///
/// // Process some samples to build up state
/// filter.process(1.0);
/// filter.process(2.0);
///
/// // Reset all internal states
/// filter.reset();
///
/// // Verify states are cleared
/// for i in 0..filter.get_n() {
///     assert_eq!(filter.get_w0()[i], 0.0);
///     assert_eq!(filter.get_w1()[i], 0.0);
///     assert_eq!(filter.get_w2()[i], 0.0);
/// }
/// ```
#[derive(Debug)]
pub struct ButterworthLPF<const N: usize> {
    /// Filter order (2, 4, 6, or 8)
    order: usize,
    /// Number of second-order sections (order/2)
    n: usize,
    /// Angular cutoff frequency (rad/s)
    wc: f32,
    /// Sample period (1/fs)
    t: f32,
    /// Gain coefficients for each section
    a: [f32; N],
    /// First feedback coefficients for each section
    d1: [f32; N],
    /// Second feedback coefficients for each section
    d2: [f32; N],
    /// Current state variables for each section
    w0: [f32; N],
    /// First delayed state variables for each section
    w1: [f32; N],
    /// Second delayed state variables for each section
    w2: [f32; N],
}

impl<const N: usize> ButterworthLPF<N> {
    /// Creates an uninitialized filter instance.
    ///
    /// The filter must be initialized using [`init`](#method.init) before use.
    /// This pattern allows for const initialization in embedded systems.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::filter::butterworth_lpf::ButterworthLPF;
    ///
    /// // Create uninitialized filter (can be const)
    /// let mut filter = ButterworthLPF::<4>::new_uninit();
    ///
    /// // Must initialize before use
    /// filter.init(4, 1000.0, 44100.0);
    /// ```
    pub const fn new_uninit() -> Self {
        Self {
            order: 0,
            n: 0,
            wc: 0.0,
            t: 0.0,
            a: [0.0; N],
            d1: [0.0; N],
            d2: [0.0; N],
            w0: [0.0; N],
            w1: [0.0; N],
            w2: [0.0; N],
        }
    }

    /// Gets the filter order.
    ///
    /// The order determines the filter's roll-off rate (20×order dB/decade) and
    /// the number of second-order sections used (order/2).
    ///
    /// # Returns
    ///
    /// The filter order (2, 4, 6, or 8).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::filter::butterworth_lpf::ButterworthLPF;
    ///
    /// let mut filter = ButterworthLPF::<4>::new_uninit();
    /// filter.init(6, 1000.0, 44100.0);
    /// assert_eq!(filter.get_order(), 6);
    /// ```
    pub fn get_order(&self) -> usize {
        self.order
    }

    /// Gets the number of second-order sections.
    ///
    /// This value is always equal to order/2 and represents the number of
    /// cascaded biquad filters used in the implementation.
    ///
    /// # Returns
    ///
    /// The number of second-order sections.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::filter::butterworth_lpf::ButterworthLPF;
    ///
    /// let mut filter = ButterworthLPF::<4>::new_uninit();
    /// filter.init(8, 1000.0, 44100.0);
    /// assert_eq!(filter.get_n(), 4); // 8th order = 4 sections
    /// ```
    pub fn get_n(&self) -> usize {
        self.n
    }

    /// Gets the current state variables (w0) for all sections.
    ///
    /// These represent the current output of each second-order section and
    /// can be useful for analysis or debugging.
    ///
    /// # Returns
    ///
    /// A slice containing the current state variables for all active sections.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::filter::butterworth_lpf::ButterworthLPF;
    ///
    /// let mut filter = ButterworthLPF::<4>::new_uninit();
    /// filter.init(4, 1000.0, 44100.0);
    ///
    /// filter.process(1.0);
    /// let states = filter.get_w0();
    /// assert_eq!(states.len(), 2); // 4th order = 2 sections
    /// ```
    pub fn get_w0(&self) -> &[f32] {
        &self.w0[..self.n]
    }

    /// Gets the first delayed state variables (w1) for all sections.
    ///
    /// These represent the previous output of each second-order section.
    ///
    /// # Returns
    ///
    /// A slice containing the first delayed state variables for all active sections.
    pub fn get_w1(&self) -> &[f32] {
        &self.w1[..self.n]
    }

    /// Gets the second delayed state variables (w2) for all sections.
    ///
    /// These represent the second previous output of each second-order section.
    ///
    /// # Returns
    ///
    /// A slice containing the second delayed state variables for all active sections.
    pub fn get_w2(&self) -> &[f32] {
        &self.w2[..self.n]
    }

    /// Initializes the filter with specified parameters.
    ///
    /// This method calculates the filter coefficients using the bilinear transformation
    /// (Tustin method) and prepares the filter for operation.
    ///
    /// # Parameters
    ///
    /// * `order` - Filter order (automatically bounded to 2-8, forced to even)
    /// * `fc` - Cutoff frequency in Hz
    /// * `fs` - Sampling frequency in Hz
    ///
    /// # Design Guidelines
    ///
    /// * `fs` should be at least 10× `fc` for good performance
    /// * `fc` must be less than `fs/2` (Nyquist limit)
    /// * Higher orders provide sharper cutoff but more delay
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::filter::butterworth_lpf::ButterworthLPF;
    ///
    /// let mut filter = ButterworthLPF::<4>::new_uninit();
    ///
    /// // Initialize for audio application
    /// filter.init(4, 1000.0, 44100.0); // 1kHz cutoff, 44.1kHz sampling
    ///
    /// // Initialize for control system
    /// filter.init(2, 100.0, 10000.0); // 100Hz cutoff, 10kHz sampling
    ///
    /// // Order bounds are automatically applied
    /// filter.init(1, 1000.0, 44100.0); // Becomes 2nd order
    /// assert_eq!(filter.get_order(), 2);
    ///
    /// filter.init(10, 1000.0, 44100.0); // Becomes 8th order  
    /// assert_eq!(filter.get_order(), 8);
    /// ```
    pub fn init(&mut self, order: usize, fc: f32, fs: f32) {
        const PI: f32 = core::f32::consts::PI;

        // Bound the order between 2 and 8 (n between 1 and 4)
        self.order = min(max(order, 2), 8);
        self.order = self.order - (self.order % 2); // Ensure order is even
        self.n = self.order / 2;

        self.t = 1.0 / fs;

        // Pre-warp cutoff frequency (Tustin transformation)
        let a = tanf(PI * fc * self.t / 2.0);
        let a2 = a * a;

        // Calculate coefficients for each section
        for i in 0..self.n {
            let r = sinf(PI * (2.0 * i as f32 + 1.0) / (4.0 * self.n as f32));
            let s = a2 + 2.0 * a * r + 1.0;

            self.a[i] = a2 / s;
            self.d1[i] = 2.0 * (1.0 - a2) / s;
            self.d2[i] = -(a2 - 2.0 * a * r + 1.0) / s;
        }
    }

    /// Resets all filter state variables to zero.
    ///
    /// This method clears the internal memory of the filter, effectively
    /// restarting it from a clean initial condition. Use this when processing
    /// a new signal or when discontinuous operation requires state reset.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::filter::butterworth_lpf::ButterworthLPF;
    ///
    /// let mut filter = ButterworthLPF::<4>::new_uninit();
    /// filter.init(4, 1000.0, 44100.0);
    ///
    /// // Process some samples
    /// filter.process(1.0);
    /// filter.process(2.0);
    ///
    /// // Reset filter state
    /// filter.reset();
    ///
    /// // All state variables should be zero
    /// for i in 0..filter.get_n() {
    ///     assert_eq!(filter.get_w0()[i], 0.0);
    ///     assert_eq!(filter.get_w1()[i], 0.0);
    ///     assert_eq!(filter.get_w2()[i], 0.0);
    /// }
    /// ```
    pub fn reset(&mut self) {
        for i in 0..self.n {
            self.w0[i] = 0.0;
            self.w1[i] = 0.0;
            self.w2[i] = 0.0;
        }
    }

    /// Processes a single input sample through the filter.
    ///
    /// This method implements the core filtering operation using the Direct Form II
    /// structure for each second-order section. The output of each section becomes
    /// the input to the next section in the cascade.
    ///
    /// # Parameters
    ///
    /// * `input` - Input sample value
    ///
    /// # Returns
    ///
    /// The filtered output sample.
    ///
    /// # Performance
    ///
    /// This method is optimized for real-time operation with:
    /// - Fixed-point arithmetic considerations
    /// - Minimal branching for predictable execution time
    /// - Cache-friendly memory access patterns
    ///
    /// # Examples
    ///
    /// ## Single Sample Processing
    ///
    /// ```rust
    /// use libpower::filter::butterworth_lpf::ButterworthLPF;
    ///
    /// let mut filter = ButterworthLPF::<4>::new_uninit();
    /// filter.init(4, 1000.0, 44100.0);
    ///
    /// let input_sample = 1.0;
    /// let output_sample = filter.process(input_sample);
    ///
    /// assert!(!output_sample.is_nan());
    /// assert!(!output_sample.is_infinite());
    /// ```
    ///
    /// ## Batch Processing
    ///
    /// ```rust
    /// use libpower::filter::butterworth_lpf::ButterworthLPF;
    ///
    /// let mut filter = ButterworthLPF::<4>::new_uninit();
    /// filter.init(4, 1000.0, 44100.0);
    ///
    /// let input_signal = [1.0, 0.5, -0.5, -1.0, 0.0];
    /// let mut output_signal = Vec::new();
    ///
    /// for &sample in &input_signal {
    ///     let filtered = filter.process(sample);
    ///     output_signal.push(filtered);
    /// }
    ///
    /// assert_eq!(output_signal.len(), input_signal.len());
    /// ```
    pub fn process(&mut self, input: f32) -> f32 {
        let mut x = input;

        // Process through each second-order section
        for i in 0..self.n {
            // Save previous states
            self.w2[i] = self.w1[i];
            self.w1[i] = self.w0[i];

            // Process current input
            self.w0[i] = self.a[i] * (x + 2.0 * self.w1[i] + self.w2[i])
                - self.d1[i] * self.w1[i]
                - self.d2[i] * self.w2[i];

            x = self.w0[i]; // Output becomes input for the next section
        }

        x
    }
}
