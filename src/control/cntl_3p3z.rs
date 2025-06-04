//! # Three-Pole Three-Zero (3P3Z) Digital Compensator
//!
//! This module implements a 3P3Z digital compensator for advanced control applications
//! requiring precise frequency response shaping. The compensator provides three poles
//! and three zeros for complex power electronics control systems.
//!
//! ## Theory
//!
//! The 3P3Z compensator implements the discrete-time transfer function:
//! ```text
//! H(z) = (b₃z³ + b₂z² + b₁z + b₀) / (z³ + a₂z² + a₁z + a₀)
//! ```
//!
//! In difference equation form:
//! ```text
//! u(k) = b₃e(k-3) + b₂e(k-2) + b₁e(k-1) + b₀e(k) + a₃u(k-3) + a₂u(k-2) + a₁u(k-1)
//! ```
//!
//! ## Applications
//!
//! - Three-phase inverter control
//! - Complex power conversion topologies
//! - Grid-tied inverter systems
//! - Advanced motor drive controllers
//! - Multi-loop control systems

/// Configuration structure for 3P3Z compensator coefficients.
///
/// This structure holds all the coefficients and limits required for the 3P3Z
/// compensator operation, including saturation limits for anti-windup protection.
#[derive(Debug, Clone)]
pub struct Coefficients {
    /// Third-order numerator coefficient (b₃)
    pub coeff_b3: f32,
    /// Second-order numerator coefficient (b₂)
    pub coeff_b2: f32,
    /// First-order numerator coefficient (b₁)
    pub coeff_b1: f32,
    /// Zero-order numerator coefficient (b₀)
    pub coeff_b0: f32,
    /// Third-order denominator coefficient (a₃)
    pub coeff_a3: f32,
    /// Second-order denominator coefficient (a₂)
    pub coeff_a2: f32,
    /// First-order denominator coefficient (a₁)
    pub coeff_a1: f32,
    /// Upper output saturation limit
    pub max: f32,
    /// Lower integrator limit (typically unused in 3P3Z)
    pub i_min: f32,
    /// Lower output saturation limit
    pub min: f32,
}

/// State variables for the 3P3Z compensator.
///
/// This structure maintains the internal state of the compensator including
/// delayed input and output values required for the difference equation implementation.
#[derive(Debug, Clone)]
pub struct Variables {
    /// Previous controller output (u(k-1))
    pub out1: f32,
    /// Second previous controller output (u(k-2))
    pub out2: f32,
    /// Third previous controller output (u(k-3))
    pub out3: f32,
    /// Current error (e(k))
    pub errn: f32,
    /// Previous error (e(k-1))
    pub errn1: f32,
    /// Second previous error (e(k-2))
    pub errn2: f32,
    /// Third previous error (e(k-3))
    pub errn3: f32,
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
            coeff_b3: 0.0,
            coeff_b2: 0.0,
            coeff_b1: 0.0,
            coeff_b0: 0.0,
            coeff_a3: 0.0,
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
            out3: 0.0,
            errn: 0.0,
            errn1: 0.0,
            errn2: 0.0,
            errn3: 0.0,
            ref_input: 0.0,
            fdbk: 0.0,
            out: 0.0,
        }
    }
}

impl Coefficients {
    /// Creates a new coefficient structure with default (zero) values.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_3p3z::Coefficients;
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
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_3p3z::Coefficients;
    ///
    /// let coeffs = Coefficients::with_default_values();
    /// assert_eq!(coeffs.coeff_b3, 0.15);
    /// assert_eq!(coeffs.coeff_a1, 0.01);
    /// ```
    pub fn with_default_values() -> Self {
        Coefficients {
            coeff_b3: 0.15,
            coeff_b2: 0.1,
            coeff_b1: 0.05,
            coeff_b0: 0.01,
            coeff_a3: 0.1,
            coeff_a2: 0.05,
            coeff_a1: 0.01,
            max: f32::MAX,
            i_min: f32::MIN,
            min: f32::MIN,
        }
    }
}

impl Variables {
    /// Creates a new variables structure with all state values set to zero.
    pub fn new() -> Self {
        Default::default()
    }

    /// Sets the reference input and feedback values.
    ///
    /// # Parameters
    ///
    /// * `ref_input` - Reference setpoint value
    /// * `fdbk` - Feedback measurement value
    pub fn set_inputs(&mut self, ref_input: f32, fdbk: f32) {
        self.ref_input = ref_input;
        self.fdbk = fdbk;
    }
}

/// Three-pole, three-zero digital compensator for advanced control applications.
///
/// The 3P3Z controller provides precise frequency response shaping through three
/// configurable poles and zeros. It's particularly effective for complex power
/// electronics control applications requiring advanced compensation.
///
/// # Examples
///
/// ## Basic Usage
///
/// ```rust
/// use libpower::control::cntl_3p3z::{Controller3p3z, Coefficients};
///
/// let coeffs = Coefficients::with_default_values();
/// let mut controller = Controller3p3z::new(coeffs);
///
/// let output = controller.calculate(1.0, 0.1);
/// assert!((controller.get_error() - 0.9).abs() < 1e-6);
/// ```
///
/// ## Reset State
///
/// ```rust
/// use libpower::control::cntl_3p3z::{Controller3p3z, Coefficients};
///
/// let coeffs = Coefficients::with_default_values();
/// let mut controller = Controller3p3z::new(coeffs);
///
/// controller.calculate(1.0, 0.1);
/// controller.reset();
/// assert_eq!(controller.get_output(), 0.0);
/// assert_eq!(controller.get_error(), 0.0);
/// ```
pub struct Controller3p3z {
    /// Compensator coefficients
    coeffs: Coefficients,
    /// Internal state variables
    vars: Variables,
}

impl Controller3p3z {
    /// Creates a new 3P3Z controller with the specified coefficients.
    ///
    /// # Parameters
    ///
    /// * `coeffs` - Compensator coefficients and saturation limits
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_3p3z::{Controller3p3z, Coefficients};
    ///
    /// let coeffs = Coefficients::with_default_values();
    /// let controller = Controller3p3z::new(coeffs);
    /// assert_eq!(controller.get_output(), 0.0);
    /// ```
    pub fn new(coeffs: Coefficients) -> Self {
        Controller3p3z {
            coeffs,
            vars: Variables::default(),
        }
    }

    /// Calculates the controller output for given reference and feedback inputs.
    ///
    /// This method implements the 3P3Z difference equation with saturation limiting.
    /// It should be called at each control sample period.
    ///
    /// # Parameters
    ///
    /// * `ref_input` - Reference setpoint value
    /// * `fdbk` - Feedback measurement value
    ///
    /// # Returns
    ///
    /// The calculated controller output, limited by saturation bounds.
    pub fn calculate(&mut self, ref_input: f32, fdbk: f32) -> f32 {
        // Update inputs
        self.vars.set_inputs(ref_input, fdbk);

        // Store previous errors
        self.vars.errn3 = self.vars.errn2;
        self.vars.errn2 = self.vars.errn1;
        self.vars.errn1 = self.vars.errn;

        // Calculate new error
        self.vars.errn = self.vars.ref_input - self.vars.fdbk;

        // Store previous outputs
        self.vars.out3 = self.vars.out2;
        self.vars.out2 = self.vars.out1;
        self.vars.out1 = self.vars.out;

        // Calculate new output
        let mut out = self.coeffs.coeff_b3 * self.vars.errn3
            + self.coeffs.coeff_b2 * self.vars.errn2
            + self.coeffs.coeff_b1 * self.vars.errn1
            + self.coeffs.coeff_b0 * self.vars.errn
            + self.coeffs.coeff_a3 * self.vars.out3
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
    pub fn get_output(&self) -> f32 {
        self.vars.out
    }

    /// Gets the current error signal.
    ///
    /// # Returns
    ///
    /// The most recent error value (ref_input - fdbk).
    pub fn get_error(&self) -> f32 {
        self.vars.errn
    }

    /// Resets all internal state variables to zero.
    ///
    /// This method clears all delayed error and output values, effectively
    /// reinitializing the controller.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_3p3z::{Controller3p3z, Coefficients};
    ///
    /// let coeffs = Coefficients::with_default_values();
    /// let mut controller = Controller3p3z::new(coeffs);
    ///
    /// controller.calculate(1.0, 0.1);
    /// controller.reset();
    /// assert_eq!(controller.get_output(), 0.0);
    /// assert_eq!(controller.get_error(), 0.0);
    /// ```
    pub fn reset(&mut self) {
        self.vars = Variables::default();
    }
}
