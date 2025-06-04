//! # Inverse Clarke Transformation (αβ0 → abc)
//!
//! This module implements the inverse Clarke transformation, which converts two-phase
//! orthogonal stationary reference frame quantities (αβ coordinates) plus a zero-sequence
//! component back into three-phase quantities (abc coordinates). This transformation
//! is the mathematical inverse of the Clarke transformation and is essential for
//! converting control signals back to three-phase systems.
//!
//! ## Theory
//!
//! The inverse Clarke transformation converts orthogonal αβ components plus zero-sequence
//! back to three symmetrical AC quantities:
//!
//! ### Mathematical Definition
//!
//! **Inverse Clarke Transform (αβ0 → abc):**
//! ```text
//! [a]   [1      0      1] [α]
//! [b] = [-1/2  √3/2   1] [β]
//! [c]   [-1/2 -√3/2   1] [0]
//! ```
//!
//! ### Expanded Form
//!
//! ```text
//! a = α + 0
//! b = -α/2 + β×(√3/2) + 0
//! c = -α/2 - β×(√3/2) + 0
//! ```
//!
//! ## Power Invariance
//!
//! The inverse transformation preserves power when properly scaled:
//! ```text
//! Pa + Pb + Pc = (3/2)(Pα + Pβ) + 3×P0
//! ```
//!
//! ## Applications
//!
//! - **Three-phase inverter control**: Converting dq control signals to abc references
//! - **Motor drive systems**: Generating three-phase voltage/current commands
//! - **Grid-tied inverters**: Converting control outputs to three-phase grid quantities
//! - **Power quality equipment**: Reconstructing three-phase signals from αβ components
//! - **Simulation and analysis**: Converting processed signals back to physical domain
//! - **SVPWM generation**: Computing three-phase duty cycles from voltage vectors

/// Inverse Clarke transformation for converting αβ0 coordinates to abc coordinates.
///
/// This structure maintains the state for inverse Clarke transformation calculations,
/// storing both input (αβ0) and output (abc) values. The transformation converts
/// two-phase orthogonal stationary reference frame quantities back to three-phase
/// balanced quantities suitable for physical three-phase systems.
///
/// # Examples
///
/// ## Basic Usage
///
/// ```rust
/// use libpower::transform::iclarke::IClarke;
///
/// // Create inverse Clarke transformer with initial αβ values
/// let mut iclarke = IClarke::new(1.0, 0.0);
///
/// // Calculate abc values
/// iclarke.calculate();
///
/// // Get the three-phase results
/// let a = iclarke.get_a();
/// let b = iclarke.get_b();
/// let c = iclarke.get_c();
///
/// println!("a: {:.3}, b: {:.3}, c: {:.3}", a, b, c);
/// // Expected: a ≈ 1.0, b ≈ -0.5, c ≈ -0.5 (balanced three-phase)
/// ```
///
/// ## Three-Phase Reconstruction
///
/// ```rust
/// use libpower::transform::iclarke::IClarke;
///
/// let mut iclarke = IClarke::new(0.0, 0.0);
///
/// // Convert various αβ combinations to abc
/// let test_cases = [
///     (1.0, 0.0),      // Pure α component
///     (0.0, 1.0),      // Pure β component  
///     (0.866, 0.5),    // 30° vector
///     (-0.5, 0.866),   // 120° vector
/// ];
///
/// for (alpha, beta) in test_cases.iter() {
///     iclarke.set_inputs(*alpha, *beta, 0.0);
///     iclarke.calculate();
///     
///     println!("α={:.3}, β={:.3} → a={:.3}, b={:.3}, c={:.3}",
///              alpha, beta, iclarke.get_a(), iclarke.get_b(), iclarke.get_c());
/// }
/// ```
///
/// ## Verifying Transformation Symmetry
///
/// ```rust
/// use libpower::transform::iclarke::IClarke;
///
/// // Test that inverse Clarke gives balanced three-phase for unit circle
/// let mut iclarke = IClarke::new(1.0, 0.0);  // Unity magnitude, 0° phase
/// iclarke.calculate();
///
/// let a = iclarke.get_a();
/// let b = iclarke.get_b();
/// let c = iclarke.get_c();
///
/// // Verify sum is zero (balanced)
/// let sum = a + b + c;
/// assert!((sum).abs() < 1e-6);
///
/// // Verify magnitudes are equal for balanced system
/// let mag_b = b.abs();
/// let mag_c = c.abs();
/// assert!((mag_b - 0.5).abs() < 1e-6);
/// assert!((mag_c - 0.5).abs() < 1e-6);
/// ```
#[derive(Debug, Clone)]
pub struct IClarke {
    /// Phase A output value
    a: f32,
    /// Phase B output value  
    b: f32,
    /// Phase C output value
    c: f32,
    /// Alpha component input (α)
    alpha: f32,
    /// Beta component input (β)
    beta: f32,
    /// Zero-sequence component input (0)
    zero: f32,
}

impl IClarke {
    /// Creates a new inverse Clarke transformer with specified αβ inputs.
    ///
    /// The transformer is initialized with the given alpha and beta components,
    /// while the zero-sequence component is set to zero and abc outputs are
    /// initialized to zero.
    ///
    /// # Parameters
    ///
    /// * `alpha` - Alpha component (α) - aligned with phase A
    /// * `beta` - Beta component (β) - orthogonal to alpha, 90° ahead
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// // Create transformer for balanced three-phase generation
    /// let iclarke = IClarke::new(1.0, 0.0);
    /// assert_eq!(iclarke.get_alpha(), 1.0);
    /// assert_eq!(iclarke.get_beta(), 0.0);
    /// assert_eq!(iclarke.get_zero(), 0.0);
    /// ```
    pub fn new(alpha: f32, beta: f32) -> IClarke {
        IClarke {
            a: 0.0,
            b: 0.0,
            c: 0.0,
            alpha,
            beta,
            zero: 0.0,
        }
    }

    /// Sets the input αβ0 components for transformation.
    ///
    /// This method allows updating all input components simultaneously before
    /// performing the inverse transformation calculation.
    ///
    /// # Parameters
    ///
    /// * `alpha` - Alpha component (α)
    /// * `beta` - Beta component (β)  
    /// * `zero` - Zero-sequence component (0)
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// let mut iclarke = IClarke::new(0.0, 0.0);
    /// iclarke.set_inputs(0.866, 0.5, 0.0);  // 30° vector
    ///
    /// assert_eq!(iclarke.get_alpha(), 0.866);
    /// assert_eq!(iclarke.get_beta(), 0.5);
    /// assert_eq!(iclarke.get_zero(), 0.0);
    /// ```
    pub fn set_inputs(&mut self, alpha: f32, beta: f32, zero: f32) {
        self.alpha = alpha;
        self.beta = beta;
        self.zero = zero;
    }

    /// Gets the Phase A output value.
    ///
    /// # Returns
    ///
    /// The calculated Phase A value after transformation.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// let mut iclarke = IClarke::new(1.0, 0.0);
    /// iclarke.calculate();
    /// assert_eq!(iclarke.get_a(), 1.0);  // Phase A = α for pure α input
    /// ```
    pub fn get_a(&self) -> f32 {
        self.a
    }

    /// Gets the Phase B output value.
    ///
    /// # Returns
    ///
    /// The calculated Phase B value after transformation.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// let mut iclarke = IClarke::new(1.0, 0.0);
    /// iclarke.calculate();
    /// assert!((iclarke.get_b() - (-0.5)).abs() < 1e-6);  // Phase B for pure α
    /// ```
    pub fn get_b(&self) -> f32 {
        self.b
    }

    /// Gets the Phase C output value.
    ///
    /// # Returns
    ///
    /// The calculated Phase C value after transformation.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// let mut iclarke = IClarke::new(1.0, 0.0);
    /// iclarke.calculate();
    /// assert!((iclarke.get_c() - (-0.5)).abs() < 1e-6);  // Phase C for pure α
    /// ```
    pub fn get_c(&self) -> f32 {
        self.c
    }

    /// Gets the current alpha component input.
    ///
    /// # Returns
    ///
    /// The current alpha component value.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// let iclarke = IClarke::new(0.866, 0.5);
    /// assert_eq!(iclarke.get_alpha(), 0.866);
    /// ```
    pub fn get_alpha(&self) -> f32 {
        self.alpha
    }

    /// Gets the current beta component input.
    ///
    /// # Returns
    ///
    /// The current beta component value.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// let iclarke = IClarke::new(0.866, 0.5);
    /// assert_eq!(iclarke.get_beta(), 0.5);
    /// ```
    pub fn get_beta(&self) -> f32 {
        self.beta
    }

    /// Gets the current zero-sequence component input.
    ///
    /// # Returns
    ///
    /// The current zero-sequence component value.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// let mut iclarke = IClarke::new(1.0, 0.0);
    /// iclarke.set_inputs(1.0, 0.0, 0.1);
    /// assert_eq!(iclarke.get_zero(), 0.1);
    /// ```
    pub fn get_zero(&self) -> f32 {
        self.zero
    }

    /// Performs the inverse Clarke transformation calculation.
    ///
    /// This method implements the mathematical inverse Clarke transformation,
    /// converting the stored αβ0 components into abc three-phase quantities.
    /// The calculation uses the standard inverse transformation equations.
    ///
    /// # Mathematical Implementation
    ///
    /// ```text
    /// a = α + 0
    /// b = -α/2 + β×(√3/2) + 0  
    /// c = -α/2 - β×(√3/2) + 0
    /// ```
    ///
    /// Where √3/2 ≈ 0.866 is approximated as 1.732/2 for computational efficiency.
    ///
    /// # Examples
    ///
    /// ## Unit Vector at 0°
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// let mut iclarke = IClarke::new(1.0, 0.0);
    /// iclarke.calculate();
    ///
    /// // Should produce standard three-phase pattern
    /// assert_eq!(iclarke.get_a(), 1.0);
    /// assert!((iclarke.get_b() - (-0.5)).abs() < 1e-6);
    /// assert!((iclarke.get_c() - (-0.5)).abs() < 1e-6);
    ///
    /// // Verify balanced (sum = 0)
    /// let sum = iclarke.get_a() + iclarke.get_b() + iclarke.get_c();
    /// assert!(sum.abs() < 1e-6);
    /// ```
    ///
    /// ## Unit Vector at 90°
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// let mut iclarke = IClarke::new(0.0, 1.0);
    /// iclarke.calculate();
    ///
    /// // Pure β input creates specific three-phase pattern
    /// assert_eq!(iclarke.get_a(), 0.0);
    /// assert!((iclarke.get_b() - 0.866).abs() < 1e-2);  // √3/2
    /// assert!((iclarke.get_c() - (-0.866)).abs() < 1e-2); // -√3/2
    /// ```
    ///
    /// ## Including Zero-Sequence
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// let mut iclarke = IClarke::new(0.0, 0.0);
    /// iclarke.set_inputs(1.0, 0.0, 0.1);  // Add zero-sequence
    /// iclarke.calculate();
    ///
    /// // Zero-sequence affects all phases equally (currently not implemented)
    /// // In full implementation: a = 1.0 + 0.1, b = -0.5 + 0.1, c = -0.5 + 0.1
    /// ```
    pub fn calculate(&mut self) {
        // Standard inverse Clarke transformation
        self.a = self.alpha;
        self.b = 0.5 * (-self.alpha + (1.732 * self.beta));
        self.c = 0.5 * (-self.alpha - (1.732 * self.beta));

        // Note: Zero-sequence component addition would be:
        // self.a += self.zero;
        // self.b += self.zero;
        // self.c += self.zero;
    }
}

impl Default for IClarke {
    /// Creates an inverse Clarke transformer with all values set to zero.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::iclarke::IClarke;
    ///
    /// let iclarke = IClarke::default();
    /// assert_eq!(iclarke.get_alpha(), 0.0);
    /// assert_eq!(iclarke.get_beta(), 0.0);
    /// assert_eq!(iclarke.get_a(), 0.0);
    /// ```
    fn default() -> Self {
        Self::new(0.0, 0.0)
    }
}
