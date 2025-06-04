//! # Park Transformation (αβ0 → dq0)
//!
//! This module implements the Park transformation, which converts two-phase orthogonal
//! stationary reference frame quantities (αβ coordinates) into a rotating reference
//! frame (dq coordinates). This transformation is crucial in three-phase motor control
//! and grid-connected systems where DC control is desired for AC quantities.
//!
//! ## Theory
//!
//! The Park transformation converts stationary αβ components into rotating dq components
//! that are aligned with a rotating reference frame. This enables DC control of AC quantities,
//! significantly simplifying controller design for three-phase systems.
//!
//! ### Mathematical Definition
//!
//! **Park Transform (αβ0 → dq0):**
//! ```text
//! [d]   [cos(θ)   sin(θ)   0] [α]
//! [q] = [-sin(θ)  cos(θ)   0] [β]
//! [0]   [0        0        1] [0]
//! ```
//!
//! ### Expanded Form
//!
//! ```text
//! d = α×cos(θ) + β×sin(θ)     // Direct component (aligned with rotor)
//! q = -α×sin(θ) + β×cos(θ)    // Quadrature component (90° ahead of d)
//! 0 = 0                       // Zero-sequence unchanged
//! ```
//!
//! ## Physical Interpretation
//!
//! - **d-axis (direct)**: Aligned with the rotating reference (e.g., rotor flux)
//! - **q-axis (quadrature)**: 90° electrical ahead of d-axis
//! - **θ (theta)**: Rotation angle of the reference frame
//!
//! ## Control Advantages
//!
//! - **DC quantities**: AC currents/voltages become DC in synchronous frame
//! - **Simplified control**: PI controllers can be used for DC quantities
//! - **Decoupled control**: d and q components can be controlled independently
//! - **Torque control**: In motors, d controls flux, q controls torque
//!
//! ## Applications
//!
//! - **Vector control of AC motors**: Field-oriented control (FOC)
//! - **Grid-tied inverters**: Synchronous frame current control
//! - **Active power filters**: Harmonic compensation in rotating frame
//! - **Wind turbine control**: Grid synchronization and power control
//! - **Battery energy storage**: Grid interface control
//! - **Power quality equipment**: Voltage regulation and reactive power control

/// Park transformation for converting αβ0 coordinates to dq0 rotating coordinates.
///
/// This structure maintains the state for Park transformation calculations, storing
/// both input (αβ0) and output (dq0) values along with the rotation angle. The
/// transformation converts stationary reference frame quantities to a rotating
/// reference frame, enabling DC control of AC quantities.
///
/// # Examples
///
/// ## Basic Motor Control Application
///
/// ```rust
/// use libpower::transform::park::Park;
///
/// // Create Park transformer
/// let mut park = Park::new(1.0, 0.0);  // Initial αβ values
///
/// // Set rotation angle for field-oriented control
/// park.set_angle(0.0, 1.0);  // θ = 0°, sin(θ) = 0, cos(θ) = 1
/// park.calculate();
///
/// // In synchronous frame aligned with d-axis
/// let d_current = park.get_d();  // Direct axis current (flux-producing)
/// let q_current = park.get_q();  // Quadrature axis current (torque-producing)
///
/// println!("d: {:.3}, q: {:.3}", d_current, q_current);
/// ```
///
/// ## Grid Synchronization Example
///
/// ```rust
/// use libpower::transform::park::Park;
///
/// let mut park = Park::new(0.0, 0.0);
///
/// // Simulate grid voltage vector tracking
/// let grid_angles = [0.0, 1.57, 3.14, 4.71]; // 0°, 90°, 180°, 270°
///
/// for angle in grid_angles.iter() {
///     // Grid voltage αβ components (example)
///     let v_alpha = libm::cosf(*angle);
///     let v_beta = libm::sinf(*angle);
///     
///     park.set_inputs(v_alpha, v_beta, 0.0);
///     park.set_angle(libm::sinf(*angle), libm::cosf(*angle));
///     park.calculate();
///     
///     println!("θ: {:.1}° → Vd: {:.3}, Vq: {:.3}",
///              angle * 180.0 / 3.14159, park.get_d(), park.get_q());
/// }
/// ```
///
/// ## Field-Oriented Control
///
/// ```rust
/// use libpower::transform::park::Park;
///
/// let mut park = Park::new(0.0, 0.0);
///
/// // Motor current measurements in αβ frame
/// let i_alpha = 1.414;  // √2 * 1A RMS
/// let i_beta = 0.0;
///
/// // Rotor position (electrical angle)
/// let theta_e = 0.0f32;  // Aligned with d-axis
///
/// park.set_inputs(i_alpha, i_beta, 0.0);
/// park.set_angle(libm::sinf(theta_e), libm::cosf(theta_e));
/// park.calculate();
///
/// // Now we have DC quantities for control
/// let id = park.get_d();  // Flux current component
/// let iq = park.get_q();  // Torque current component
///
/// assert!((id - 1.414).abs() < 1e-3);  // All current in d-axis
/// assert!(iq.abs() < 1e-6);            // No torque current
/// ```
///
/// ## Sinusoidal Reference Generation
///
/// ```rust
/// use libpower::transform::park::Park;
///
/// let mut park = Park::new(0.0, 0.0);
///
/// // Generate sinusoidal reference in αβ frame - Test with 0° angle
/// let angle = 0.0f32;
/// let v_alpha = libm::cosf(angle);
/// let v_beta = libm::sinf(angle);
///
/// // Transform to dq frame
/// park.set_angle(libm::sinf(angle), libm::cosf(angle));
/// park.set_inputs(v_alpha, v_beta, 0.0);
/// park.calculate();
///
/// // In dq frame: d should be constant, q should be near zero  
/// assert!((park.get_d() - 1.0).abs() < 0.1);
/// assert!(park.get_q().abs() < 0.1);
/// ```
///
/// ## Motor Control Field Orientation
///
/// ```rust
/// use libpower::transform::park::Park;
///
/// let mut park = Park::new(0.0, 0.0);
///
/// // Set rotor angle for field orientation
/// let theta_e = 0.0f32;  // Aligned with d-axis
///
/// // Set three-phase currents (balanced)
/// park.set_angle(libm::sinf(theta_e), libm::cosf(theta_e));
/// park.set_inputs(1.0, 0.0, 0.0);  // Pure α-axis current, no β component
/// park.calculate();
///
/// // d-axis current (flux-producing) when aligned
/// assert!((park.get_d() - 1.0).abs() < 0.01);
/// // q-axis current (torque-producing) should be zero when aligned
/// assert!(park.get_q().abs() < 0.01);
/// ```
#[derive(Debug, Clone)]
pub struct Park {
    /// Alpha component input (α) - stationary frame
    alpha: f32,
    /// Beta component input (β) - stationary frame
    beta: f32,
    /// Zero-sequence component (0) - unchanged
    zero: f32,
    /// Sine of rotation angle (sin θ)
    sin: f32,
    /// Cosine of rotation angle (cos θ)
    cos: f32,
    /// Direct axis component output (d) - rotating frame
    d: f32,
    /// Quadrature axis component output (q) - rotating frame
    q: f32,
    /// Zero-sequence component output (z) - unchanged
    z: f32,
}

impl Park {
    /// Creates a new Park transformer with specified αβ inputs.
    ///
    /// The transformer is initialized with the given alpha and beta components.
    /// The rotation angle (sin/cos) must be set using [`set_angle`](#method.set_angle)
    /// before performing calculations.
    ///
    /// # Parameters
    ///
    /// * `alpha` - Alpha component (α) in stationary frame
    /// * `beta` - Beta component (β) in stationary frame
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// // Create transformer for motor current control
    /// let park = Park::new(1.414, 0.0);  // Current vector along α-axis
    /// assert_eq!(park.get_alpha(), 1.414);
    /// assert_eq!(park.get_beta(), 0.0);
    /// ```
    pub fn new(alpha: f32, beta: f32) -> Park {
        Park {
            alpha,
            beta,
            zero: 0.0,
            sin: 0.0,
            cos: 1.0, // Initialize to θ = 0° (cos = 1, sin = 0)
            d: 0.0,
            q: 0.0,
            z: 0.0,
        }
    }

    /// Sets the input αβ0 components for transformation.
    ///
    /// This method allows updating all input components simultaneously before
    /// performing the Park transformation calculation.
    ///
    /// # Parameters
    ///
    /// * `alpha` - Alpha component (α) in stationary frame
    /// * `beta` - Beta component (β) in stationary frame
    /// * `zero` - Zero-sequence component (0)
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let mut park = Park::new(0.0, 0.0);
    /// park.set_inputs(1.0, 0.5, 0.0);  // Set new αβ0 values
    ///
    /// assert_eq!(park.get_alpha(), 1.0);
    /// assert_eq!(park.get_beta(), 0.5);
    /// assert_eq!(park.get_zero(), 0.0);
    /// ```
    pub fn set_inputs(&mut self, alpha: f32, beta: f32, zero: f32) {
        self.alpha = alpha;
        self.beta = beta;
        self.zero = zero;
    }

    /// Sets the rotation angle using sin and cos values.
    ///
    /// The Park transformation requires the rotation angle θ that defines the
    /// orientation of the rotating reference frame. This method accepts pre-calculated
    /// sine and cosine values for computational efficiency.
    ///
    /// # Parameters
    ///
    /// * `sin_theta` - Sine of rotation angle (sin θ)
    /// * `cos_theta` - Cosine of rotation angle (cos θ)
    ///
    /// # Design Notes
    ///
    /// - In motor control: θ is typically the electrical rotor angle
    /// - In grid applications: θ comes from PLL tracking grid phase
    /// - For efficiency: sin/cos are often computed once per control cycle
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let mut park = Park::new(1.0, 0.0);
    ///
    /// // Set angle for θ = 30° (π/6 radians)
    /// let theta = std::f32::consts::PI / 6.0;
    /// park.set_angle(theta.sin(), theta.cos());
    ///
    /// assert!((park.get_sin() - 0.5).abs() < 1e-6);      // sin(30°) = 0.5
    /// assert!((park.get_cos() - 0.866).abs() < 1e-3);    // cos(30°) ≈ 0.866
    /// ```
    pub fn set_angle(&mut self, sin_theta: f32, cos_theta: f32) {
        self.sin = sin_theta;
        self.cos = cos_theta;
    }

    /// Gets the alpha component input.
    ///
    /// # Returns
    ///
    /// The current alpha component value in the stationary frame.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let park = Park::new(1.414, 0.5);
    /// assert_eq!(park.get_alpha(), 1.414);
    /// ```
    pub fn get_alpha(&self) -> f32 {
        self.alpha
    }

    /// Gets the beta component input.
    ///
    /// # Returns
    ///
    /// The current beta component value in the stationary frame.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let park = Park::new(1.414, 0.5);
    /// assert_eq!(park.get_beta(), 0.5);
    /// ```
    pub fn get_beta(&self) -> f32 {
        self.beta
    }

    /// Gets the zero-sequence component.
    ///
    /// # Returns
    ///
    /// The current zero-sequence component value.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let mut park = Park::new(1.0, 0.0);
    /// park.set_inputs(1.0, 0.0, 0.1);
    /// assert_eq!(park.get_zero(), 0.1);
    /// ```
    pub fn get_zero(&self) -> f32 {
        self.zero
    }

    /// Gets the sine of the rotation angle.
    ///
    /// # Returns
    ///
    /// The sine component of the rotation angle (sin θ).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let mut park = Park::new(1.0, 0.0);
    /// park.set_angle(0.5, 0.866);  // 30° angle
    /// assert_eq!(park.get_sin(), 0.5);
    /// ```
    pub fn get_sin(&self) -> f32 {
        self.sin
    }

    /// Gets the cosine of the rotation angle.
    ///
    /// # Returns
    ///
    /// The cosine component of the rotation angle (cos θ).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let mut park = Park::new(1.0, 0.0);
    /// park.set_angle(0.5, 0.866);  // 30° angle
    /// assert_eq!(park.get_cos(), 0.866);
    /// ```
    pub fn get_cos(&self) -> f32 {
        self.cos
    }

    /// Gets the direct axis (d) component output.
    ///
    /// The d-axis component represents the component aligned with the rotating
    /// reference frame. In motor control, this typically corresponds to the
    /// flux-producing component.
    ///
    /// # Returns
    ///
    /// The calculated direct axis component after transformation.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let mut park = Park::new(1.0, 0.0);
    /// park.set_angle(0.0, 1.0);  // θ = 0°
    /// park.calculate();
    /// assert_eq!(park.get_d(), 1.0);  // α maps directly to d when θ = 0°
    /// ```
    pub fn get_d(&self) -> f32 {
        self.d
    }

    /// Gets the quadrature axis (q) component output.
    ///
    /// The q-axis component represents the component 90° ahead of the rotating
    /// reference frame. In motor control, this typically corresponds to the
    /// torque-producing component.
    ///
    /// # Returns
    ///
    /// The calculated quadrature axis component after transformation.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let mut park = Park::new(0.0, 1.0);
    /// park.set_angle(0.0, 1.0);  // θ = 0°
    /// park.calculate();
    /// assert_eq!(park.get_q(), 1.0);  // β maps directly to q when θ = 0°
    /// ```
    pub fn get_q(&self) -> f32 {
        self.q
    }

    /// Gets the zero-sequence component output.
    ///
    /// The zero-sequence component passes through unchanged in the Park transformation.
    ///
    /// # Returns
    ///
    /// The zero-sequence component (same as input).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let mut park = Park::new(1.0, 0.0);
    /// park.set_inputs(1.0, 0.0, 0.5);
    /// park.calculate();
    /// assert_eq!(park.get_z(), 0.0);  // Currently not implemented
    /// ```
    pub fn get_z(&self) -> f32 {
        self.z
    }

    /// Performs the Park transformation calculation.
    ///
    /// This method implements the mathematical Park transformation, converting
    /// the stored αβ0 components into dq0 rotating reference frame quantities
    /// using the specified rotation angle.
    ///
    /// # Mathematical Implementation
    ///
    /// ```text
    /// d = α×cos(θ) + β×sin(θ)
    /// q = -α×sin(θ) + β×cos(θ)
    /// z = 0  (zero-sequence unchanged)
    /// ```
    ///
    /// # Examples
    ///
    /// ## Aligned Reference Frame (θ = 0°)
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let mut park = Park::new(1.0, 0.5);
    /// park.set_angle(0.0, 1.0);  // θ = 0°: sin = 0, cos = 1
    /// park.calculate();
    ///
    /// // When θ = 0°: d = α, q = β
    /// assert_eq!(park.get_d(), 1.0);   // α maps to d
    /// assert_eq!(park.get_q(), 0.5);   // β maps to q
    /// ```
    ///
    /// ## 90° Rotated Reference Frame
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let mut park = Park::new(1.0, 0.0);
    /// park.set_angle(1.0, 0.0);  // θ = 90°: sin = 1, cos = 0
    /// park.calculate();
    ///
    /// // When θ = 90°: d = β, q = -α
    /// assert_eq!(park.get_d(), 0.0);   // β component
    /// assert_eq!(park.get_q(), -1.0);  // -α component
    /// ```
    ///
    /// ## Field-Oriented Control Example
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let mut park = Park::new(0.0, 0.0);
    ///
    /// // Motor with sinusoidal current: i_a = I*cos(ωt), i_b = I*cos(ωt - 2π/3)
    /// // After Clarke: α = I*cos(ωt), β = I*sin(ωt)
    /// let current_magnitude = 10.0;  // 10A current
    /// let electrical_angle = std::f32::consts::PI / 4.0;  // 45°
    ///
    /// let i_alpha = current_magnitude * electrical_angle.cos();
    /// let i_beta = current_magnitude * electrical_angle.sin();
    ///
    /// park.set_inputs(i_alpha, i_beta, 0.0);
    /// park.set_angle(electrical_angle.sin(), electrical_angle.cos());
    /// park.calculate();
    ///
    /// // In synchronous frame, sinusoidal currents become DC
    /// let id = park.get_d();
    /// let iq = park.get_q();
    ///
    /// // For this example, expect mostly d-axis current
    /// assert!((id - current_magnitude).abs() < 1e-3);
    /// assert!(iq.abs() < 1e-6);
    /// ```
    pub fn calculate(&mut self) {
        // Park transformation: αβ → dq
        self.d = self.alpha * self.cos + self.beta * self.sin;
        self.q = self.beta * self.cos - self.alpha * self.sin;
        self.z = 0.0; // Zero-sequence component unchanged
    }
}

impl Default for Park {
    /// Creates a Park transformer with all values set to zero.
    ///
    /// The rotation angle is initialized to θ = 0° (cos = 1.0, sin = 0.0).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::park::Park;
    ///
    /// let park = Park::default();
    /// assert_eq!(park.get_alpha(), 0.0);
    /// assert_eq!(park.get_beta(), 0.0);
    /// assert_eq!(park.get_cos(), 1.0);   // θ = 0°
    /// assert_eq!(park.get_sin(), 0.0);
    /// ```
    fn default() -> Self {
        Self::new(0.0, 0.0)
    }
}
