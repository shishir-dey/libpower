//! # Chebyshev Type I High-Pass Filter
//!
//! This module implements a digital Chebyshev Type I high-pass filter using cascaded
//! second-order sections (SOS). Chebyshev filters provide steeper roll-off compared
//! to Butterworth filters but with ripple in the passband, making them suitable for
//! applications requiring sharp DC blocking and tight transition bands.
//!
//! ## Theory
//!
//! The Chebyshev Type I high-pass filter is characterized by:
//! - **Equiripple passband**: Controlled ripple in the passband (determined by ε)
//! - **Monotonic stopband**: Smooth rolloff without oscillations in stopband
//! - **Steeper roll-off**: Better than Butterworth for same order
//! - **DC blocking**: Infinite attenuation at DC (0 Hz)
//! - **Trade-off**: Passband ripple vs. transition bandwidth
//!
//! ## Transfer Function
//!
//! Each second-order section implements:
//! ```text
//! H(z) = a₀(1 - 2z⁻¹ + z⁻²) / (1 + d₁z⁻¹ + d₂z⁻²)
//! ```
//!
//! Where coefficients are derived from Chebyshev polynomials with s → 1/s
//! transformation for high-pass response, then converted to z-domain.
//!
//! ## Design Parameters
//!
//! - **ε (epsilon)**: Controls passband ripple (typical: 0.1 to 1.0)
//!   - Smaller ε → Less ripple, closer to Butterworth response
//!   - Larger ε → More ripple, steeper transition band
//! - **Ripple in dB**: 20*log₁₀(√(1 + ε²))
//!
//! ## Applications
//!
//! - DC blocking with sharp cutoff requirements
//! - High-frequency noise analysis
//! - AC coupling with minimal signal distortion
//! - Power quality analysis (removing DC offset)
//! - Audio applications (subsonic content removal)
//! - EMI filtering for low-frequency rejection

use libm::{cosf, coshf, logf, sinf, sinhf, sqrtf, tanf};

/// Chebyshev Type I high-pass filter implemented using cascaded second-order sections.
///
/// This filter provides steeper roll-off than Butterworth filters but with controlled
/// ripple in the passband. The implementation uses even orders (2, 4, 6, 8) and
/// requires specification of the ripple parameter (epsilon).
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
/// use libpower::filter::chebyshev_hpf::ChebyshevHPF;
///
/// // Create and initialize a 4th-order Chebyshev high-pass filter
/// let mut filter = ChebyshevHPF::<4>::new_uninit();
/// let result = filter.init(4, 0.1, 44100.0, 100.0); // 4th order, ε=0.1, 44.1kHz sampling, 100Hz cutoff
/// assert!(result.is_ok());
///
/// // Process a sample
/// let output = filter.process(1.0);
/// assert!(!output.is_nan());
/// ```
///
/// ## DC Blocking Application
///
/// ```rust
/// use libpower::filter::chebyshev_hpf::ChebyshevHPF;
///
/// let mut filter = ChebyshevHPF::<4>::new_uninit();
/// let result = filter.init(2, 0.5, 10000.0, 10.0); // Sharp 10Hz high-pass
/// assert!(result.is_ok());
///
/// // Feed DC signal - should be heavily attenuated
/// let mut dc_output = 0.0;
/// for _ in 0..1000 {
///     dc_output = filter.process(1.0); // DC input
/// }
///
/// // DC should be heavily attenuated
/// assert!(dc_output.abs() < 0.1);
/// ```
///
/// ## Error Handling
///
/// ```rust
/// use libpower::filter::chebyshev_hpf::ChebyshevHPF;
///
/// let mut filter = ChebyshevHPF::<4>::new_uninit();
///
/// // Invalid parameters return errors
/// assert!(filter.init(3, 0.1, 44100.0, 1000.0).is_err()); // Odd order
/// assert!(filter.init(4, 0.0, 44100.0, 1000.0).is_err()); // Invalid epsilon
/// assert!(filter.init(4, 0.1, 44100.0, 25000.0).is_err()); // fc > fs/2
/// ```
#[derive(Debug)]
pub struct ChebyshevHPF<const N: usize> {
    /// Number of second-order sections (order/2)
    m: usize,
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
    /// Epsilon normalization factor for output scaling
    ep: f32,
}

impl<const N: usize> ChebyshevHPF<N> {
    /// Creates an uninitialized filter instance.
    ///
    /// The filter must be initialized using [`init`](#method.init) before use.
    /// This pattern allows for const initialization in embedded systems.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::filter::chebyshev_hpf::ChebyshevHPF;
    ///
    /// // Create uninitialized filter (can be const)
    /// let mut filter = ChebyshevHPF::<4>::new_uninit();
    ///
    /// // Must initialize before use
    /// let result = filter.init(4, 0.1, 44100.0, 100.0);
    /// assert!(result.is_ok());
    /// ```
    pub const fn new_uninit() -> Self {
        Self {
            m: 0,
            a: [0.0; N],
            d1: [0.0; N],
            d2: [0.0; N],
            w0: [0.0; N],
            w1: [0.0; N],
            w2: [0.0; N],
            ep: 0.0,
        }
    }

    /// Gets the number of second-order sections.
    ///
    /// This value is equal to order/2 and represents the number of
    /// cascaded biquad filters used in the implementation.
    ///
    /// # Returns
    ///
    /// The number of second-order sections.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::filter::chebyshev_hpf::ChebyshevHPF;
    ///
    /// let mut filter = ChebyshevHPF::<4>::new_uninit();
    /// let result = filter.init(6, 0.1, 44100.0, 100.0);
    /// assert!(result.is_ok());
    /// assert_eq!(filter.get_m(), 3); // 6th order = 3 sections
    /// ```
    pub fn get_m(&self) -> usize {
        self.m
    }

    /// Gets the current state variables (w0) for all sections.
    ///
    /// These represent the current output of each second-order section and
    /// can be useful for analysis or debugging.
    ///
    /// # Returns
    ///
    /// A slice containing the current state variables for all active sections.
    pub fn get_w0(&self) -> &[f32] {
        &self.w0[..self.m]
    }

    /// Gets the first delayed state variables (w1) for all sections.
    ///
    /// These represent the previous output of each second-order section.
    ///
    /// # Returns
    ///
    /// A slice containing the first delayed state variables for all active sections.
    pub fn get_w1(&self) -> &[f32] {
        &self.w1[..self.m]
    }

    /// Gets the second delayed state variables (w2) for all sections.
    ///
    /// These represent the second previous output of each second-order section.
    ///
    /// # Returns
    ///
    /// A slice containing the second delayed state variables for all active sections.
    pub fn get_w2(&self) -> &[f32] {
        &self.w2[..self.m]
    }

    /// Initializes the filter with specified parameters.
    ///
    /// This method calculates the filter coefficients using Chebyshev polynomial
    /// design with high-pass transformation and bilinear transformation to z-domain.
    ///
    /// # Parameters
    ///
    /// * `order` - Filter order (must be even, 2-8 for N=4)
    /// * `epsilon` - Ripple parameter (must be > 0, typical: 0.01-1.0)
    /// * `sample_rate` - Sampling frequency in Hz
    /// * `cutoff_freq` - Cutoff frequency in Hz (must be < sample_rate/2)
    ///
    /// # Returns
    ///
    /// - `Ok(())` if initialization succeeds
    /// - `Err(&'static str)` with error description if parameters are invalid
    ///
    /// # Errors
    ///
    /// - "Order must be even" - Odd filter orders are not supported
    /// - "Order too large for allocated size" - Order exceeds N*2
    /// - "Epsilon must be positive" - Ripple parameter ≤ 0
    /// - "Invalid cutoff frequency" - fc ≤ 0 or fc ≥ fs/2
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::filter::chebyshev_hpf::ChebyshevHPF;
    ///
    /// let mut filter = ChebyshevHPF::<4>::new_uninit();
    ///
    /// // Valid initialization for DC blocking
    /// let result = filter.init(4, 0.1, 10000.0, 10.0);
    /// assert!(result.is_ok());
    ///
    /// // Invalid parameters
    /// assert!(filter.init(3, 0.1, 44100.0, 100.0).is_err()); // Odd order
    /// assert!(filter.init(4, 0.0, 44100.0, 100.0).is_err()); // Zero epsilon
    /// assert!(filter.init(4, 0.1, 1000.0, 600.0).is_err());  // fc > fs/2
    /// ```
    pub fn init(
        &mut self,
        order: usize,
        epsilon: f32,
        sample_rate: f32,
        cutoff_freq: f32,
    ) -> Result<(), &'static str> {
        if order % 2 != 0 {
            return Err("Order must be even");
        }
        if order > N * 2 {
            return Err("Order too large for allocated size");
        }
        if epsilon <= 0.0 {
            return Err("Epsilon must be positive");
        }
        if cutoff_freq <= 0.0 || cutoff_freq >= sample_rate / 2.0 {
            return Err("Invalid cutoff frequency");
        }

        self.m = order / 2;
        const PI: f32 = core::f32::consts::PI;

        // Prewarp cutoff frequency
        let a = tanf(PI * cutoff_freq / sample_rate);
        let a2 = a * a;

        // Calculate filter parameters
        let u = logf(1.0 + sqrtf(1.0 + epsilon * epsilon) / epsilon);
        let su = sinhf(u / order as f32);
        let cu = coshf(u / order as f32);

        // Calculate coefficients for each section
        for i in 0..self.m {
            let b = sinf(PI * (2.0 * i as f32 + 1.0) / (2.0 * order as f32)) * su;
            let c = cosf(PI * (2.0 * i as f32 + 1.0) / (2.0 * order as f32)) * cu;
            let c = b * b + c * c;

            // High-pass specific coefficient calculations
            let s = a2 + 2.0 * a * b + c;
            self.a[i] = 1.0 / (4.0 * s);
            self.d1[i] = 2.0 * (c - a2) / s;
            self.d2[i] = -(a2 - 2.0 * a * b + c) / s;
        }

        self.ep = 2.0 / epsilon;
        self.reset();

        Ok(())
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
    /// use libpower::filter::chebyshev_hpf::ChebyshevHPF;
    ///
    /// let mut filter = ChebyshevHPF::<4>::new_uninit();
    /// let result = filter.init(4, 0.1, 44100.0, 100.0);
    /// assert!(result.is_ok());
    ///
    /// // Process some samples
    /// filter.process(1.0);
    /// filter.process(2.0);
    ///
    /// // Reset filter state
    /// filter.reset();
    ///
    /// // All state variables should be zero
    /// for i in 0..filter.get_m() {
    ///     assert_eq!(filter.get_w0()[i], 0.0);
    ///     assert_eq!(filter.get_w1()[i], 0.0);
    ///     assert_eq!(filter.get_w2()[i], 0.0);
    /// }
    /// ```
    pub fn reset(&mut self) {
        for i in 0..self.m {
            self.w0[i] = 0.0;
            self.w1[i] = 0.0;
            self.w2[i] = 0.0;
        }
    }

    /// Processes a single input sample through the filter.
    ///
    /// This method implements the core filtering operation for each second-order
    /// section in the cascade. The final output is scaled by the epsilon
    /// normalization factor and implements the high-pass response.
    ///
    /// # Parameters
    ///
    /// * `input` - Input sample value
    ///
    /// # Returns
    ///
    /// The filtered output sample with high-frequency content preserved and
    /// low-frequency content attenuated according to the Chebyshev response.
    ///
    /// # Examples
    ///
    /// ## Basic Processing
    ///
    /// ```rust
    /// use libpower::filter::chebyshev_hpf::ChebyshevHPF;
    ///
    /// let mut filter = ChebyshevHPF::<4>::new_uninit();
    /// let result = filter.init(4, 0.1, 44100.0, 100.0);
    /// assert!(result.is_ok());
    ///
    /// let input_sample = 1.0;
    /// let output_sample = filter.process(input_sample);
    ///
    /// assert!(!output_sample.is_nan());
    /// assert!(!output_sample.is_infinite());
    /// ```
    ///
    /// ## DC Blocking Example
    ///
    /// ```rust
    /// use libpower::filter::chebyshev_hpf::ChebyshevHPF;
    ///
    /// let mut filter = ChebyshevHPF::<4>::new_uninit();
    /// let result = filter.init(2, 0.1, 10000.0, 0.5); // Very low cutoff
    /// assert!(result.is_ok());
    ///
    /// // Process DC signal - should approach zero
    /// let mut dc_output = 0.0;
    /// for _ in 0..5000 {
    ///     dc_output = filter.process(1.0); // Constant DC input
    /// }
    ///
    /// // DC should be heavily attenuated
    /// assert!(dc_output.abs() < 0.3);
    /// ```
    pub fn process(&mut self, input: f32) -> f32 {
        let mut output = input;

        // Process through each second-order section
        for i in 0..self.m {
            self.w0[i] = self.d1[i] * self.w1[i] + self.d2[i] * self.w2[i] + output;
            output = self.a[i] * (self.w0[i] - 2.0 * self.w1[i] + self.w2[i]); // Note the sign change for high-pass
            self.w2[i] = self.w1[i];
            self.w1[i] = self.w0[i];
        }

        output * self.ep
    }
}
