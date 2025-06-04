//! # Clarke Transformation (abc → αβ0)
//!
//! This module implements the Clarke transformation, which converts three-phase
//! quantities (abc coordinates) into a two-phase orthogonal stationary reference
//! frame (αβ coordinates) plus a zero-sequence component. This transformation
//! is fundamental in three-phase power electronics and motor control applications.
//!
//! ## Theory
//!
//! The Clarke transformation converts three symmetrical AC quantities into two
//! orthogonal components plus a zero-sequence component:
//!
//! ### Mathematical Definition
//!
//! **Forward Clarke Transform (abc → αβ0):**
//! ```text
//! [α]   [1     -1/2    -1/2 ] [a]
//! [β] = [0   √3/2   -√3/2 ] [b] × (2/3)
//! [0]   [1/2   1/2     1/2 ] [c]
//! ```
//!
//! ### Expanded Form
//!
//! ```text
//! α = (2/3) × (a - b/2 - c/2)      // Aligned with phase A
//! β = (2/3) × (√3/2) × (b - c)     // Orthogonal to α (90° ahead)
//! 0 = (1/3) × (a + b + c)          // Zero-sequence (unbalanced component)
//! ```
//!
//! ## Power Invariance
//!
//! The transformation preserves instantaneous power:
//! ```text
//! Pa + Pb + Pc = (3/2)(Pα + Pβ) + 3×P0
//! ```
//!
//! ## Physical Interpretation
//!
//! - **α-axis**: Aligned with phase A axis
//! - **β-axis**: 90° ahead of α-axis in electrical degrees
//! - **0-sequence**: Common mode component (indicates unbalance)
//!
//! ## Applications
//!
//! - **Vector control of AC motors**: Foundation for field-oriented control
//! - **Grid-tied inverters**: Converting three-phase measurements to αβ frame
//! - **Power quality analysis**: Decomposing three-phase unbalanced quantities
//! - **Space vector PWM**: Generating optimal switching patterns
//! - **Active power filters**: Harmonic analysis and compensation
//! - **Wind turbine control**: Grid synchronization and power control

use libm::{atan2f, sqrtf};

/// Three-phase to two-phase Clarke transformation.
///
/// This structure provides both stateful and stateless Clarke transformation
/// operations. The stateful version maintains internal state for repeated
/// calculations, while the stateless version provides utility functions
/// for one-time calculations.
///
/// # Examples
///
/// ## Stateful Operation
///
/// ```rust
/// use libpower::transform::clarke::Clarke;
///
/// // Create transformer and set inputs
/// let mut clarke = Clarke::new(0.0, 0.0);
/// clarke.set_inputs(100.0, -50.0, -50.0);  // Balanced three-phase
///
/// // Perform transformation  
/// clarke.calculate_stateful();
///
/// // Access results
/// let alpha = clarke.get_alpha();
/// let beta = clarke.get_beta();
/// let zero = clarke.get_zero();
///
/// println!("α: {:.2}, β: {:.2}, 0: {:.2}", alpha, beta, zero);
/// assert!(zero.abs() < 1e-6);  // Should be balanced (zero ≈ 0)
/// ```
///
/// ## Stateless Operation
///
/// ```rust
/// use libpower::transform::clarke::Clarke;
///
/// let clarke = Clarke::new(0.0, 0.0);
/// let result = clarke.calculate(100.0, -50.0, -50.0);
///
/// println!("α: {:.2}, β: {:.2}, 0: {:.2}",
///          result.alpha, result.beta, result.zero);
/// ```
///
/// ## Motor Control Application
///
/// ```rust
/// use libpower::transform::clarke::Clarke;
///
/// let mut clarke = Clarke::new(0.0, 0.0);
///
/// // Three-phase current measurements
/// let ia = 10.0;   // Phase A current
/// let ib = -5.0;   // Phase B current  
/// let ic = -5.0;   // Phase C current
///
/// clarke.set_inputs(ia, ib, ic);
/// clarke.calculate_stateful();
///
/// // Use α-β currents for space vector control
/// let i_alpha = clarke.get_alpha();
/// let i_beta = clarke.get_beta();
///
/// // Check if system is balanced
/// let zero_seq = clarke.get_zero();
/// let is_balanced = zero_seq.abs() < 1.0;  // 1A tolerance
///
/// assert!(is_balanced);
/// ```
#[derive(Debug, Clone)]
pub struct Clarke {
    /// Phase A input value
    a: f32,
    /// Phase B input value
    b: f32,
    /// Phase C input value
    c: f32,
    /// Alpha component output (α)
    alpha: f32,
    /// Beta component output (β)
    beta: f32,
    /// Zero-sequence component output (0)
    zero: f32,
}

/// Result structure for Clarke transformation calculations.
///
/// This structure holds the results of a Clarke transformation calculation,
/// providing the α, β, and zero-sequence components.
#[derive(Debug, Clone, Copy)]
pub struct ClarkeResult {
    /// Alpha component (α) - aligned with phase A
    pub alpha: f32,
    /// Beta component (β) - orthogonal to alpha (90° ahead)
    pub beta: f32,
    /// Zero-sequence component (0) - common mode/unbalanced component
    pub zero: f32,
}

impl Clarke {
    /// Creates a new Clarke transformation with specified initial α and β values.
    ///
    /// The transformer is initialized with the given alpha and beta components.
    /// The abc inputs must be set using [`set_inputs`](#method.set_inputs) before
    /// performing stateful calculations.
    ///
    /// # Parameters
    ///
    /// * `alpha` - Initial alpha component value
    /// * `beta` - Initial beta component value
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    ///
    /// let clarke = Clarke::new(1.0, 0.5);
    /// assert_eq!(clarke.get_alpha(), 1.0);
    /// assert_eq!(clarke.get_beta(), 0.5);
    /// ```
    pub fn new(alpha: f32, beta: f32) -> Clarke {
        Clarke {
            a: 0.0,
            b: 0.0,
            c: 0.0,
            alpha,
            beta,
            zero: 0.0,
        }
    }

    /// Sets the three-phase input values for transformation.
    ///
    /// This method updates the internal abc values that will be used in the
    /// next call to [`calculate_stateful`](#method.calculate_stateful).
    ///
    /// # Parameters
    ///
    /// * `a` - Phase A value (voltage, current, etc.)
    /// * `b` - Phase B value
    /// * `c` - Phase C value
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    ///
    /// let mut clarke = Clarke::new(0.0, 0.0);
    /// clarke.set_inputs(100.0, -50.0, -50.0);
    ///
    /// assert_eq!(clarke.get_a(), 100.0);
    /// assert_eq!(clarke.get_b(), -50.0);
    /// assert_eq!(clarke.get_c(), -50.0);
    /// ```
    pub fn set_inputs(&mut self, a: f32, b: f32, c: f32) {
        self.a = a;
        self.b = b;
        self.c = c;
    }

    /// Gets the Phase A input value.
    ///
    /// # Returns
    ///
    /// The current Phase A input value.
    pub fn get_a(&self) -> f32 {
        self.a
    }

    /// Gets the Phase B input value.
    ///
    /// # Returns
    ///
    /// The current Phase B input value.
    pub fn get_b(&self) -> f32 {
        self.b
    }

    /// Gets the Phase C input value.
    ///
    /// # Returns
    ///
    /// The current Phase C input value.
    pub fn get_c(&self) -> f32 {
        self.c
    }

    /// Gets the alpha component output.
    ///
    /// The alpha component is aligned with the Phase A axis and represents
    /// the primary component of the space vector.
    ///
    /// # Returns
    ///
    /// The current alpha component value.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    ///
    /// let mut clarke = Clarke::new(0.0, 0.0);
    /// clarke.set_inputs(100.0, 0.0, 0.0);  // Pure Phase A
    /// clarke.calculate_stateful();
    ///
    /// let alpha = clarke.get_alpha();
    /// assert!(alpha > 0.0);  // Should be positive for positive Phase A
    /// ```
    pub fn get_alpha(&self) -> f32 {
        self.alpha
    }

    /// Gets the beta component output.
    ///
    /// The beta component is orthogonal to the alpha component (90° ahead)
    /// and represents the quadrature component of the space vector.
    ///
    /// # Returns
    ///
    /// The current beta component value.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    ///
    /// let mut clarke = Clarke::new(0.0, 0.0);
    /// clarke.set_inputs(0.0, 100.0, -100.0);  // Pure B-C differential
    /// clarke.calculate_stateful();
    ///
    /// let beta = clarke.get_beta();
    /// assert!(beta > 0.0);  // Should be positive for B > C
    /// ```
    pub fn get_beta(&self) -> f32 {
        self.beta
    }

    /// Gets the zero-sequence component output.
    ///
    /// The zero-sequence component represents the common-mode or unbalanced
    /// component of the three-phase system. For a balanced system, this
    /// value should be close to zero.
    ///
    /// # Returns
    ///
    /// The current zero-sequence component value.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    ///
    /// let mut clarke = Clarke::new(0.0, 0.0);
    ///
    /// // Balanced system
    /// clarke.set_inputs(100.0, -50.0, -50.0);
    /// clarke.calculate_stateful();
    /// assert!(clarke.get_zero().abs() < 1e-6);
    ///
    /// // Unbalanced system
    /// clarke.set_inputs(100.0, 0.0, 0.0);
    /// clarke.calculate_stateful();
    /// assert!(clarke.get_zero().abs() > 10.0);
    /// ```
    pub fn get_zero(&self) -> f32 {
        self.zero
    }

    /// Performs the stateful Clarke transformation calculation.
    ///
    /// This method computes the Clarke transformation using the internally stored
    /// abc values (set via [`set_inputs`](#method.set_inputs)) and updates the
    /// internal α, β, and zero-sequence components.
    ///
    /// # Mathematical Implementation
    ///
    /// Uses the power-invariant Clarke transformation:
    /// ```text
    /// α = (2/3) × (a - b/2 - c/2)
    /// β = (2/3) × (√3/2) × (b - c)  
    /// 0 = (1/3) × (a + b + c)
    /// ```
    ///
    /// # Examples
    ///
    /// ## Balanced Three-Phase System
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    ///
    /// let mut clarke = Clarke::new(0.0, 0.0);
    ///
    /// // 120° phase-shifted sinusoidal system
    /// clarke.set_inputs(100.0, -50.0, -50.0);
    /// clarke.calculate_stateful();
    ///
    /// let alpha = clarke.get_alpha();
    /// let beta = clarke.get_beta();
    /// let zero = clarke.get_zero();
    ///
    /// // Balanced system: zero-sequence ≈ 0
    /// assert!(zero.abs() < 1e-6);
    ///
    /// // Check magnitude preservation (approximately)
    /// let abc_rms = (100.0_f32.powi(2) + 50.0_f32.powi(2) + 50.0_f32.powi(2)).sqrt() / 3.0_f32.sqrt();
    /// let ab_rms = (alpha.powi(2) + beta.powi(2)).sqrt() / (3.0_f32/2.0).sqrt();
    /// ```
    ///
    /// ## Unbalanced System Detection
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    ///
    /// let mut clarke = Clarke::new(0.0, 0.0);
    ///
    /// // Unbalanced system with DC offset
    /// clarke.set_inputs(110.0, -40.0, -50.0);  // Sum ≠ 0
    /// clarke.calculate_stateful();
    ///
    /// let zero_seq = clarke.get_zero();
    /// assert!(zero_seq.abs() > 5.0);  // Significant unbalance
    ///
    /// println!("Unbalance detected: {:.2}", zero_seq);
    /// ```
    pub fn calculate_stateful(&mut self) {
        // Power-invariant Clarke transformation
        // Constants for transformation
        const TWO_THIRDS: f32 = 2.0 / 3.0;
        const SQRT3_OVER_2: f32 = 0.8660254037844387; // √3/2
        const ONE_THIRD: f32 = 1.0 / 3.0;

        // Calculate α component: (2/3) * (a - b/2 - c/2)
        self.alpha = TWO_THIRDS * (self.a - 0.5 * self.b - 0.5 * self.c);

        // Calculate β component: (2/3) * (√3/2) * (b - c)
        self.beta = TWO_THIRDS * SQRT3_OVER_2 * (self.b - self.c);

        // Calculate zero-sequence component: (1/3) * (a + b + c)
        self.zero = ONE_THIRD * (self.a + self.b + self.c);
    }

    /// Performs Clarke transformation on given abc values (stateless).
    ///
    /// This method computes the Clarke transformation for the provided abc inputs
    /// without modifying the internal state. It returns a `ClarkeResult` structure
    /// containing the calculated components.
    ///
    /// # Parameters
    ///
    /// * `a` - Phase A quantity
    /// * `b` - Phase B quantity
    /// * `c` - Phase C quantity
    ///
    /// # Returns
    ///
    /// A `ClarkeResult` containing the α, β, and zero-sequence components.
    ///
    /// # Examples
    ///
    /// ## Basic Transformation
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    ///
    /// let clarke = Clarke::new(0.0, 0.0);
    /// let result = clarke.calculate(100.0, -50.0, -50.0);
    ///
    /// println!("α: {:.2}, β: {:.2}, 0: {:.2}",
    ///          result.alpha, result.beta, result.zero);
    ///
    /// // Verify balanced system
    /// assert!(result.zero.abs() < 1e-6);
    /// ```
    ///
    /// ## Real-time Processing
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    /// use std::f32::consts::PI;
    ///
    /// let clarke = Clarke::new(0.0, 0.0);
    /// let mut results = Vec::new();
    ///
    /// // Process samples from three-phase system
    /// for i in 0..10 {
    ///     let t = i as f32 * 0.001; // 1ms time steps
    ///     let omega = 2.0 * PI * 50.0; // 50 Hz
    ///     
    ///     // Generate three-phase sinusoidal signals
    ///     let v_a = 325.0 * (omega * t).sin();
    ///     let v_b = 325.0 * (omega * t - 2.0 * PI / 3.0).sin();
    ///     let v_c = 325.0 * (omega * t + 2.0 * PI / 3.0).sin();
    ///     
    ///     let clarke_result = clarke.calculate(v_a, v_b, v_c);
    ///     results.push(clarke_result);
    /// }
    ///
    /// // Process results for control algorithm
    /// for result in results {
    ///     // Use α-β components for space vector control
    ///     let magnitude = (result.alpha.powi(2) + result.beta.powi(2)).sqrt();
    ///     let angle = clarke.angle(&result);
    /// }
    /// ```
    pub fn calculate(&self, a: f32, b: f32, c: f32) -> ClarkeResult {
        // Power-invariant Clarke transformation
        // Constants for transformation
        const TWO_THIRDS: f32 = 2.0 / 3.0;
        const SQRT3_OVER_2: f32 = 0.8660254037844387; // √3/2
        const ONE_THIRD: f32 = 1.0 / 3.0;

        // Calculate α component: (2/3) * (a - b/2 - c/2)
        let alpha = TWO_THIRDS * (a - 0.5 * b - 0.5 * c);

        // Calculate β component: (2/3) * (√3/2) * (b - c)
        let beta = TWO_THIRDS * SQRT3_OVER_2 * (b - c);

        // Calculate zero-sequence component: (1/3) * (a + b + c)
        let zero = ONE_THIRD * (a + b + c);

        ClarkeResult { alpha, beta, zero }
    }

    /// Performs Clarke transformation using amplitude-invariant form.
    ///
    /// This variant preserves the amplitude of the original signals rather than power.
    /// The peak values of α and β components equal the peak values of the abc components.
    ///
    /// # Parameters
    ///
    /// * `a` - Phase A quantity
    /// * `b` - Phase B quantity  
    /// * `c` - Phase C quantity
    ///
    /// # Returns
    ///
    /// A `ClarkeResult` with amplitude-invariant α, β, and zero components.
    ///
    /// # Mathematical Implementation
    ///
    /// ```text
    /// α = a - b/2 - c/2
    /// β = (√3/2) × (b - c)
    /// 0 = (1/3) × (a + b + c)
    /// ```
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    ///
    /// let clarke = Clarke::new(0.0, 0.0);
    ///
    /// // Compare power-invariant vs amplitude-invariant
    /// let (a, b, c) = (100.0, -50.0, -50.0);
    ///
    /// let power_inv = clarke.calculate(a, b, c);
    /// let amp_inv = clarke.calculate_amplitude_invariant(a, b, c);
    ///
    /// // Amplitude-invariant gives larger values
    /// assert!(amp_inv.alpha > power_inv.alpha);
    /// println!("Power-inv α: {:.1}, Amp-inv α: {:.1}",
    ///          power_inv.alpha, amp_inv.alpha);
    /// ```
    pub fn calculate_amplitude_invariant(&self, a: f32, b: f32, c: f32) -> ClarkeResult {
        // Amplitude-invariant Clarke transformation
        const SQRT3_OVER_2: f32 = 0.8660254037844387; // √3/2
        const ONE_THIRD: f32 = 1.0 / 3.0;

        // Calculate α component: a - b/2 - c/2
        let alpha = a - 0.5 * b - 0.5 * c;

        // Calculate β component: (√3/2) * (b - c)
        let beta = SQRT3_OVER_2 * (b - c);

        // Calculate zero-sequence component: (1/3) * (a + b + c)
        let zero = ONE_THIRD * (a + b + c);

        ClarkeResult { alpha, beta, zero }
    }

    /// Calculates the magnitude of the αβ vector.
    ///
    /// Computes the magnitude of the space vector represented by the α and β components.
    /// This is useful for determining the total magnitude of the three-phase quantity.
    ///
    /// # Parameters
    ///
    /// * `result` - Clarke transformation result
    ///
    /// # Returns
    ///
    /// The magnitude of the αβ space vector.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    ///
    /// let clarke = Clarke::new(0.0, 0.0);
    /// let result = clarke.calculate(100.0, -50.0, -50.0);
    /// let magnitude = clarke.magnitude(&result);
    ///
    /// assert!(magnitude > 0.0);
    /// println!("Space vector magnitude: {:.2}", magnitude);
    /// ```
    pub fn magnitude(&self, result: &ClarkeResult) -> f32 {
        sqrtf(result.alpha * result.alpha + result.beta * result.beta)
    }

    /// Calculates the phase angle of the αβ vector.
    ///
    /// Computes the angle of the space vector in the αβ plane using atan2.
    /// The angle is measured from the α-axis towards the β-axis.
    ///
    /// # Parameters
    ///
    /// * `result` - Clarke transformation result
    ///
    /// # Returns
    ///
    /// The phase angle in radians [-π, π].
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    /// use std::f32::consts::PI;
    ///
    /// let clarke = Clarke::new(0.0, 0.0);
    /// let result = clarke.calculate(100.0, -50.0, -50.0);
    /// let angle = clarke.angle(&result);
    ///
    /// // Angle should be between -π and π
    /// assert!(angle >= -PI && angle <= PI);
    /// println!("Space vector angle: {:.2} rad ({:.1}°)",
    ///          angle, angle * 180.0 / PI);
    /// ```
    pub fn angle(&self, result: &ClarkeResult) -> f32 {
        atan2f(result.beta, result.alpha)
    }

    /// Checks if the three-phase system is balanced.
    ///
    /// A balanced three-phase system should have a zero-sequence component
    /// close to zero. This method checks if the zero-sequence is within
    /// a specified tolerance.
    ///
    /// # Parameters
    ///
    /// * `result` - Clarke transformation result
    /// * `tolerance` - Maximum allowed zero-sequence magnitude for balanced system
    ///
    /// # Returns
    ///
    /// `true` if the system is considered balanced, `false` otherwise.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::clarke::Clarke;
    ///
    /// let clarke = Clarke::new(0.0, 0.0);
    ///
    /// // Balanced system
    /// let balanced = clarke.calculate(100.0, -50.0, -50.0);
    /// assert!(clarke.is_balanced(&balanced, 1.0));
    ///
    /// // Unbalanced system  
    /// let unbalanced = clarke.calculate(120.0, -50.0, -50.0);
    /// assert!(!clarke.is_balanced(&unbalanced, 1.0));
    /// ```
    pub fn is_balanced(&self, result: &ClarkeResult, tolerance: f32) -> bool {
        result.zero.abs() <= tolerance
    }
}

impl Default for Clarke {
    /// Creates a new Clarke transformation with default parameters.
    ///
    /// This is equivalent to [`new(0.0, 0.0)`](#method.new) but follows the standard
    /// Rust convention for default initialization.
    fn default() -> Self {
        Self::new(0.0, 0.0)
    }
}
