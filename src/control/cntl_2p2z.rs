//! # Two-Pole Two-Zero (2P2Z) Digital Compensator
//!
//! This module implements a 2P2Z digital compensator commonly used in power electronics
//! control systems. The compensator provides two poles and two zeros for precise frequency
//! response shaping in voltage and current control loops.
//!
//! ## Theory
//!
//! The 2P2Z compensator implements the discrete-time transfer function:
//! ```text
//! H(z) = (b₂z² + b₁z + b₀) / (z² + a₁z + a₂)
//! ```
//!
//! In difference equation form:
//! ```text
//! u(k) = b₂e(k-2) + b₁e(k-1) + b₀e(k) + a₂u(k-2) + a₁u(k-1)
//! ```
//!
//! Where:
//! - `b₂, b₁, b₀` are the numerator coefficients (zeros)
//! - `a₂, a₁` are the denominator coefficients (poles)
//! - `e(k)` is the error signal at sample k
//! - `u(k)` is the controller output at sample k
//!
//! ## Design Applications
//!
//! ### DC-DC Converter Voltage Loops
//! The 2P2Z compensator is ideal for voltage loop compensation in:
//! - Buck converters
//! - Boost converters  
//! - Buck-boost converters
//! - Flyback converters
//!
//! ### Typical Frequency Response Shaping
//! - **Low Frequency**: High gain for good regulation
//! - **Crossover Frequency**: Unity gain with adequate phase margin
//! - **High Frequency**: Sufficient attenuation for noise immunity
//!
//! ## Coefficient Calculation
//!
//! Coefficients are typically calculated using:
//! 1. Analog compensator design (s-domain)
//! 2. Bilinear transformation (Tustin method)
//! 3. Digital control design tools
//!
//! ## Applications
//!
//! - Switch-mode power supply voltage loops
//! - Battery charger regulation
//! - LED driver control
//! - Motor drive DC bus regulation

/// Configuration structure for 2P2Z compensator coefficients.
///
/// This structure holds all the coefficients and limits required for the 2P2Z
/// compensator operation, including saturation limits for anti-windup protection.
///
/// # Examples
///
/// ## Default Coefficients
///
/// ```rust
/// use libpower::control::cntl_2p2z::Coefficients;
///
/// let coeffs = Coefficients::new();
/// // All coefficients initialized to 0.0
/// assert_eq!(coeffs.coeff_b0, 0.0);
/// assert_eq!(coeffs.coeff_a1, 0.0);
/// ```
///
/// ## Pre-configured Coefficients
///
/// ```rust
/// use libpower::control::cntl_2p2z::Coefficients;
///
/// let coeffs = Coefficients::with_default_values();
/// assert_eq!(coeffs.coeff_b2, 0.3);
/// assert_eq!(coeffs.coeff_b1, 0.2);
/// assert_eq!(coeffs.coeff_b0, 0.1);
/// ```
#[derive(Debug, Clone)]
pub struct Coefficients {
    /// Second-order numerator coefficient (b₂)
    pub coeff_b2: f32,
    /// First-order numerator coefficient (b₁)
    pub coeff_b1: f32,
    /// Zero-order numerator coefficient (b₀)
    pub coeff_b0: f32,
    /// Second-order denominator coefficient (a₂)
    pub coeff_a2: f32,
    /// First-order denominator coefficient (a₁)
    pub coeff_a1: f32,
    /// Upper output saturation limit
    pub max: f32,
    /// Lower integrator limit (typically unused in 2P2Z)
    pub i_min: f32,
    /// Lower output saturation limit
    pub min: f32,
}

/// State variables for the 2P2Z compensator.
///
/// This structure maintains the internal state of the compensator including
/// delayed input and output values required for the difference equation implementation.
///
/// # Examples
///
/// ```rust
/// use libpower::control::cntl_2p2z::Variables;
///
/// let mut vars = Variables::new();
/// vars.set_inputs(5.0, 4.8); // reference = 5.0, feedback = 4.8
/// assert_eq!(vars.ref_input, 5.0);
/// assert_eq!(vars.fdbk, 4.8);
/// ```
#[derive(Debug, Clone)]
pub struct Variables {
    /// Previous controller output (u(k-1))
    pub out1: f32,
    /// Second previous controller output (u(k-2))
    pub out2: f32,
    /// Current error (e(k))
    pub errn: f32,
    /// Previous error (e(k-1))
    pub errn1: f32,
    /// Second previous error (e(k-2))
    pub errn2: f32,
    /// Reference input signal
    pub ref_input: f32,
    /// Feedback signal
    pub fdbk: f32,
    /// Current controller output
    pub out: f32,
}

impl Default for Coefficients {
    /// Creates coefficients with all values set to zero and no saturation limits.
    fn default() -> Self {
        Coefficients {
            coeff_b2: 0.0,
            coeff_b1: 0.0,
            coeff_b0: 0.0,
            coeff_a2: 0.0,
            coeff_a1: 0.0,
            max: f32::MAX,
            i_min: f32::MIN,
            min: f32::MIN,
        }
    }
}

impl Default for Variables {
    /// Creates variables with all state values initialized to zero.
    fn default() -> Self {
        Variables {
            out1: 0.0,
            out2: 0.0,
            errn: 0.0,
            errn1: 0.0,
            errn2: 0.0,
            ref_input: 0.0,
            fdbk: 0.0,
            out: 0.0,
        }
    }
}

impl Coefficients {
    /// Creates a new coefficient structure with default (zero) values.
    ///
    /// This is equivalent to using `Default::default()` but provides a more
    /// explicit constructor pattern.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_2p2z::Coefficients;
    ///
    /// let coeffs = Coefficients::new();
    /// assert_eq!(coeffs.coeff_b0, 0.0);
    /// assert_eq!(coeffs.max, f32::MAX);
    /// ```
    pub fn new() -> Self {
        Default::default()
    }

    /// Creates coefficients with example values suitable for testing and initial design.
    ///
    /// These coefficients provide a stable compensator response and can be used as
    /// a starting point for system identification and tuning procedures.
    ///
    /// # Coefficient Values
    /// - b₂ = 0.3, b₁ = 0.2, b₀ = 0.1 (numerator)
    /// - a₂ = 0.2, a₁ = 0.1 (denominator)
    /// - No saturation limits (±f32::MAX)
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_2p2z::Coefficients;
    ///
    /// let coeffs = Coefficients::with_default_values();
    /// assert_eq!(coeffs.coeff_b2, 0.3);
    /// assert_eq!(coeffs.coeff_b1, 0.2);
    /// assert_eq!(coeffs.coeff_b0, 0.1);
    /// assert_eq!(coeffs.coeff_a2, 0.2);
    /// assert_eq!(coeffs.coeff_a1, 0.1);
    /// ```
    pub fn with_default_values() -> Self {
        Coefficients {
            coeff_b2: 0.3,
            coeff_b1: 0.2,
            coeff_b0: 0.1,
            coeff_a2: 0.2,
            coeff_a1: 0.1,
            max: f32::MAX,
            i_min: f32::MIN,
            min: f32::MIN,
        }
    }
}

impl Variables {
    /// Creates a new variables structure with all state values set to zero.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_2p2z::Variables;
    ///
    /// let vars = Variables::new();
    /// assert_eq!(vars.out, 0.0);
    /// assert_eq!(vars.errn, 0.0);
    /// ```
    pub fn new() -> Self {
        Default::default()
    }

    /// Sets the reference input and feedback values.
    ///
    /// This is a convenience method for updating both input signals simultaneously.
    /// The error calculation is performed separately in the controller.
    ///
    /// # Parameters
    ///
    /// * `ref_input` - Reference setpoint value
    /// * `fdbk` - Feedback measurement value
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_2p2z::Variables;
    ///
    /// let mut vars = Variables::new();
    /// vars.set_inputs(10.0, 9.5);
    /// assert_eq!(vars.ref_input, 10.0);
    /// assert_eq!(vars.fdbk, 9.5);
    /// ```
    pub fn set_inputs(&mut self, ref_input: f32, fdbk: f32) {
        self.ref_input = ref_input;
        self.fdbk = fdbk;
    }
}

/// Two-pole, two-zero digital compensator for advanced control applications.
///
/// The 2P2Z controller provides precise frequency response shaping through two
/// configurable poles and zeros. It's particularly effective for voltage loop
/// compensation in switch-mode power supplies and other power electronics applications.
///
/// # Examples
///
/// ## Basic Usage
///
/// ```rust
/// use libpower::control::cntl_2p2z::{Controller2p2z, Coefficients};
///
/// // Create controller with default coefficients
/// let coeffs = Coefficients::with_default_values();
/// let mut controller = Controller2p2z::new(coeffs);
///
/// // Execute control calculation
/// let output = controller.calculate(1.0, 0.1); // ref=1.0, feedback=0.1
/// assert!((controller.get_error() - 0.9).abs() < 1e-6);
/// ```
///
/// ## With Saturation Limits
///
/// ```rust
/// use libpower::control::cntl_2p2z::{Controller2p2z, Coefficients};
///
/// let mut coeffs = Coefficients::with_default_values();
/// coeffs.max = 1.0;  // +1.0 upper limit
/// coeffs.min = -1.0; // -1.0 lower limit
///
/// let mut controller = Controller2p2z::new(coeffs);
/// let output = controller.calculate(10.0, 0.0); // Large error
/// assert!(output <= 1.0 && output >= -1.0);
/// ```
///
/// ## Reset State
///
/// ```rust
/// use libpower::control::cntl_2p2z::{Controller2p2z, Coefficients};
///
/// let coeffs = Coefficients::with_default_values();
/// let mut controller = Controller2p2z::new(coeffs);
///
/// // Accumulate some state
/// controller.calculate(1.0, 0.1);
/// assert!(controller.get_output() != 0.0);
///
/// // Reset all state variables
/// controller.reset();
/// assert_eq!(controller.get_output(), 0.0);
/// assert_eq!(controller.get_error(), 0.0);
/// ```
pub struct Controller2p2z {
    /// Compensator coefficients
    coeffs: Coefficients,
    /// Internal state variables
    vars: Variables,
}

impl Controller2p2z {
    /// Creates a new 2P2Z controller with the specified coefficients.
    ///
    /// The controller is initialized with zero state variables and is ready
    /// for immediate use in control loops.
    ///
    /// # Parameters
    ///
    /// * `coeffs` - Compensator coefficients and saturation limits
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_2p2z::{Controller2p2z, Coefficients};
    ///
    /// let coeffs = Coefficients::with_default_values();
    /// let controller = Controller2p2z::new(coeffs);
    /// assert_eq!(controller.get_output(), 0.0);
    /// ```
    pub fn new(coeffs: Coefficients) -> Self {
        Controller2p2z {
            coeffs,
            vars: Variables::default(),
        }
    }

    /// Calculates the controller output for given reference and feedback inputs.
    ///
    /// This method implements the 2P2Z difference equation with saturation limiting.
    /// It should be called at each control sample period with the current reference
    /// and feedback values.
    ///
    /// # Parameters
    ///
    /// * `ref_input` - Reference setpoint value
    /// * `fdbk` - Feedback measurement value
    ///
    /// # Returns
    ///
    /// The calculated controller output, limited by saturation bounds.
    ///
    /// # Implementation
    ///
    /// The algorithm implements:
    /// 1. Error calculation: e(k) = ref - fdbk
    /// 2. State history update: e(k-1)→e(k-2), u(k-1)→u(k-2)
    /// 3. Difference equation evaluation
    /// 4. Saturation limiting
    /// 5. State variable updates
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_2p2z::{Controller2p2z, Coefficients};
    ///
    /// let coeffs = Coefficients::with_default_values();
    /// let mut controller = Controller2p2z::new(coeffs);
    ///
    /// // Control loop execution
    /// for _ in 0..10 {
    ///     let output = controller.calculate(5.0, 4.8);
    ///     // Use output for actuation
    ///     assert!(!output.is_nan());
    /// }
    /// ```
    pub fn calculate(&mut self, ref_input: f32, fdbk: f32) -> f32 {
        // Update inputs
        self.vars.set_inputs(ref_input, fdbk);

        // Store previous errors
        self.vars.errn2 = self.vars.errn1;
        self.vars.errn1 = self.vars.errn;

        // Calculate new error
        self.vars.errn = self.vars.ref_input - self.vars.fdbk;

        // Store previous outputs
        self.vars.out2 = self.vars.out1;
        self.vars.out1 = self.vars.out;

        // Calculate new output
        let mut out = self.coeffs.coeff_b2 * self.vars.errn2
            + self.coeffs.coeff_b1 * self.vars.errn1
            + self.coeffs.coeff_b0 * self.vars.errn
            + self.coeffs.coeff_a2 * self.vars.out2
            + self.coeffs.coeff_a1 * self.vars.out1;

        // Apply saturation limits
        out = out.min(self.coeffs.max).max(self.coeffs.min);

        self.vars.out = out;
        out
    }

    /// Gets the current controller output.
    ///
    /// # Returns
    ///
    /// The most recent controller output value, after saturation limiting.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_2p2z::{Controller2p2z, Coefficients};
    ///
    /// let coeffs = Coefficients::with_default_values();
    /// let mut controller = Controller2p2z::new(coeffs);
    ///
    /// controller.calculate(1.0, 0.5);
    /// let output = controller.get_output();
    /// assert!(!output.is_nan());
    /// ```
    pub fn get_output(&self) -> f32 {
        self.vars.out
    }

    /// Gets the current error signal.
    ///
    /// The error is calculated as: error = reference - feedback
    ///
    /// # Returns
    ///
    /// The most recent error value (ref_input - fdbk).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_2p2z::{Controller2p2z, Coefficients};
    ///
    /// let coeffs = Coefficients::with_default_values();
    /// let mut controller = Controller2p2z::new(coeffs);
    ///
    /// controller.calculate(1.0, 0.1);
    /// assert!((controller.get_error() - 0.9).abs() < 1e-6);
    /// ```
    pub fn get_error(&self) -> f32 {
        self.vars.errn
    }

    /// Resets all internal state variables to zero.
    ///
    /// This method clears all delayed error and output values, effectively
    /// reinitializing the controller. Use this when starting a new control
    /// sequence or when discontinuous operation requires state reset.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_2p2z::{Controller2p2z, Coefficients};
    ///
    /// let coeffs = Coefficients::with_default_values();
    /// let mut controller = Controller2p2z::new(coeffs);
    ///
    /// // Accumulate some state
    /// controller.calculate(1.0, 0.1);
    /// assert!(controller.get_output() != 0.0);
    ///
    /// // Reset controller
    /// controller.reset();
    /// assert_eq!(controller.get_output(), 0.0);
    /// assert_eq!(controller.get_error(), 0.0);
    /// ```
    pub fn reset(&mut self) {
        self.vars = Variables::default();
    }
}
