//! # Inverse Park Transformation (dq0 → αβ0)
//!
//! This module implements the inverse Park transformation, which converts rotating
//! reference frame quantities (dq coordinates) back into two-phase orthogonal
//! stationary reference frame quantities (αβ coordinates). This transformation
//! is essential for converting control signals from the rotating frame back to
//! the stationary frame for implementation in three-phase systems.
//!
//! ## Theory
//!
//! The inverse Park transformation converts rotating dq components back into stationary
//! αβ components. This is the mathematical inverse of the Park transformation and is
//! crucial for implementing field-oriented control where control is performed in the
//! dq frame but actuation must be in the abc frame.
//!
//! ### Mathematical Definition
//!
//! **Inverse Park Transform (dq0 → αβ0):**
//! ```text
//! [α]   [cos(θ)  -sin(θ)  0] [d]
//! [β] = [sin(θ)   cos(θ)  0] [q]
//! [0]   [0        0       1] [0]
//! ```
//!
//! ### Expanded Form
//!
//! ```text
//! α = d×cos(θ) - q×sin(θ)     // Alpha component in stationary frame
//! β = d×sin(θ) + q×cos(θ)     // Beta component in stationary frame  
//! 0 = 0                       // Zero-sequence unchanged
//! ```
//!
//! ## Physical Interpretation
//!
//! - **d component**: DC magnitude aligned with rotating reference
//! - **q component**: DC magnitude 90° ahead of rotating reference
//! - **θ (theta)**: Rotation angle of the reference frame
//! - **α, β outputs**: Sinusoidal quantities in stationary frame
//!
//! ## Control Flow in Field-Oriented Control
//!
//! 1. **Measurement**: abc → Clarke → αβ → Park → dq (DC quantities)
//! 2. **Control**: PI controllers operate on DC dq quantities
//! 3. **Actuation**: dq → Inverse Park → αβ → Inverse Clarke → abc
//!
//! ## Applications
//!
//! - **Vector control of AC motors**: Converting dq voltage commands to αβ
//! - **Grid-tied inverters**: Converting dq current references to αβ
//! - **Active power filters**: Converting rotating frame commands to stationary
//! - **Wind turbine control**: Grid synchronization and power control
//! - **Battery energy storage**: Grid interface voltage/current generation
//! - **Motor drive PWM generation**: Creating three-phase modulation signals

/// Inverse Park transformation for converting dq0 coordinates to αβ0 stationary coordinates.
///
/// This structure maintains the state for inverse Park transformation calculations,
/// storing both input (dq0) and output (αβ0) values along with the rotation angle.
/// The transformation converts rotating reference frame quantities back to stationary
/// reference frame quantities, enabling physical implementation of control commands.
///
/// # Examples
///
/// ## Basic Motor Control Application
///
/// ```rust
/// use libpower::transform::ipark::IPark;
///
/// // Create inverse Park transformer
/// let mut ipark = IPark::new(0.0, 0.0);
///
/// // Set dq control outputs from PI controllers
/// ipark.set_inputs(10.0, 5.0, 0.0);  // id = 10A, iq = 5A (DC values)
///
/// // Set rotation angle from rotor position
/// let theta = std::f32::consts::PI / 6.0;  // 30° electrical
/// ipark.set_angle(theta.sin(), theta.cos());
/// ipark.calculate();
///
/// // Get αβ voltages for PWM generation
/// let v_alpha = ipark.get_alpha();  // Stationary frame voltage
/// let v_beta = ipark.get_beta();    // Stationary frame voltage
///
/// println!("v_α: {:.3}, v_β: {:.3}", v_alpha, v_beta);
/// ```
///
/// ## Field-Oriented Control Example
///
/// ```rust
/// use libpower::transform::ipark::IPark;
///
/// let mut ipark = IPark::new(0.0, 0.0);
///
/// // Motor control scenario: pure torque command (q-axis only)
/// let id_ref = 0.0;   // No flux current (field weakening off)
/// let iq_ref = 15.0;  // 15A torque current command
///
/// ipark.set_inputs(id_ref, iq_ref, 0.0);
///
/// // Rotor position from encoder/estimator
/// let rotor_angle = 0.0f32;  // Aligned with d-axis
/// ipark.set_angle(rotor_angle.sin(), rotor_angle.cos());
/// ipark.calculate();
///
/// // Pure q-axis current becomes pure β-axis current when θ = 0°
/// assert_eq!(ipark.get_alpha(), 0.0);   // No α component
/// assert_eq!(ipark.get_beta(), 15.0);   // All current in β
/// ```
///
/// ## Sinusoidal Generation
///
/// ```rust
/// use libpower::transform::ipark::IPark;
///
/// let mut ipark = IPark::new(0.0, 0.0);
///
/// // Generate sinusoidal output by rotating a constant dq vector
/// let magnitude = 100.0;  // 100V amplitude
/// let d_component = magnitude;  // Constant d-axis
/// let q_component = 0.0;        // No q-axis
///
/// ipark.set_inputs(d_component, q_component, 0.0);
///
/// // Simulate rotation over one electrical cycle
/// for step in 0..8 {
///     let angle = step as f32 * std::f32::consts::PI / 4.0;  // 45° steps
///     ipark.set_angle(angle.sin(), angle.cos());
///     ipark.calculate();
///     
///     println!("θ: {:.0}° → α: {:.1}, β: {:.1}",
///              angle * 180.0 / std::f32::consts::PI,
///              ipark.get_alpha(), ipark.get_beta());
/// }
/// // This generates: α = magnitude×cos(θ), β = magnitude×sin(θ)
/// ```
///
/// ## Motor Control Voltage Generation
///
/// ```rust
/// use libpower::transform::ipark::IPark;
///
/// let mut ipark = IPark::new(0.0, 0.0);
///
/// // Set desired d-q voltages (typical FOC)
/// ipark.set_inputs(10.0, 5.0, 0.0);  // Vd=10V, Vq=5V
///
/// let rotor_angle = 0.0f32;  // Aligned with d-axis
/// ipark.set_angle(rotor_angle.sin(), rotor_angle.cos());
/// ipark.calculate();
///
/// // α-axis should equal d-axis when aligned (θ=0)
/// assert!((ipark.get_alpha() - 10.0).abs() < 0.01);
/// // β-axis should equal q-axis when aligned
/// assert!((ipark.get_beta() - 5.0).abs() < 0.01);
/// ```
#[derive(Debug, Clone)]
pub struct IPark {
    /// Alpha component output (α) - stationary frame
    alpha: f32,
    /// Beta component output (β) - stationary frame
    beta: f32,
    /// Zero-sequence component (0) - unchanged
    zero: f32,
    /// Sine of rotation angle (sin θ)
    sin: f32,
    /// Cosine of rotation angle (cos θ)
    cos: f32,
    /// Direct axis component input (d) - rotating frame
    d: f32,
    /// Quadrature axis component input (q) - rotating frame
    q: f32,
    /// Zero-sequence component input (z) - unchanged
    z: f32,
}

impl IPark {
    /// Creates a new inverse Park transformer with specified αβ initial values.
    ///
    /// The transformer is initialized with the given alpha and beta components
    /// for compatibility with the existing API. The rotation angle (sin/cos) and
    /// dq inputs must be set before performing calculations.
    ///
    /// # Parameters
    ///
    /// * `alpha` - Initial alpha component (not used for dq → αβ transformation)
    /// * `beta` - Initial beta component (not used for dq → αβ transformation)
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// // Create transformer (initial αβ values are overwritten by calculation)
    /// let ipark = IPark::new(0.0, 0.0);
    /// assert_eq!(ipark.get_d(), 0.0);
    /// assert_eq!(ipark.get_q(), 0.0);
    /// ```
    pub fn new(alpha: f32, beta: f32) -> IPark {
        IPark {
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

    /// Sets the input dq0 components for transformation.
    ///
    /// This method sets the rotating frame quantities that will be converted
    /// back to stationary frame quantities. These are typically the outputs
    /// from dq-frame controllers.
    ///
    /// # Parameters
    ///
    /// * `d` - Direct axis component (d) - typically flux-related
    /// * `q` - Quadrature axis component (q) - typically torque-related
    /// * `z` - Zero-sequence component (z) - unchanged
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    /// ipark.set_inputs(10.0, 5.0, 0.0);  // Set dq control outputs
    ///
    /// assert_eq!(ipark.get_d(), 10.0);
    /// assert_eq!(ipark.get_q(), 5.0);
    /// assert_eq!(ipark.get_z(), 0.0);
    /// ```
    pub fn set_inputs(&mut self, d: f32, q: f32, z: f32) {
        self.d = d;
        self.q = q;
        self.z = z;
    }

    /// Sets the rotation angle using sin and cos values.
    ///
    /// The inverse Park transformation requires the same rotation angle θ used
    /// in the forward Park transformation. This method accepts pre-calculated
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
    /// - Must be the same angle used in forward Park transformation
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    ///
    /// // Set angle for θ = 60° (π/3 radians)
    /// let theta = std::f32::consts::PI / 3.0;
    /// ipark.set_angle(theta.sin(), theta.cos());
    ///
    /// assert!((ipark.get_sin() - 0.866).abs() < 1e-3);    // sin(60°) ≈ 0.866
    /// assert!((ipark.get_cos() - 0.5).abs() < 1e-6);      // cos(60°) = 0.5
    /// ```
    pub fn set_angle(&mut self, sin_theta: f32, cos_theta: f32) {
        self.sin = sin_theta;
        self.cos = cos_theta;
    }

    /// Gets the alpha component output.
    ///
    /// # Returns
    ///
    /// The calculated alpha component value in the stationary frame.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    /// ipark.set_inputs(10.0, 0.0, 0.0);  // Pure d-axis
    /// ipark.set_angle(0.0, 1.0);         // θ = 0°
    /// ipark.calculate();
    /// assert_eq!(ipark.get_alpha(), 10.0);  // d maps to α when θ = 0°
    /// ```
    pub fn get_alpha(&self) -> f32 {
        self.alpha
    }

    /// Gets the beta component output.
    ///
    /// # Returns
    ///
    /// The calculated beta component value in the stationary frame.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    /// ipark.set_inputs(0.0, 10.0, 0.0);  // Pure q-axis
    /// ipark.set_angle(0.0, 1.0);         // θ = 0°
    /// ipark.calculate();
    /// assert_eq!(ipark.get_beta(), 10.0);   // q maps to β when θ = 0°
    /// ```
    pub fn get_beta(&self) -> f32 {
        self.beta
    }

    /// Gets the zero-sequence component.
    ///
    /// # Returns
    ///
    /// The zero-sequence component value (currently unused).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    /// ipark.set_inputs(1.0, 1.0, 0.5);
    /// assert_eq!(ipark.get_zero(), 0.0);  // Currently not implemented
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
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    /// ipark.set_angle(0.707, 0.707);  // 45° angle
    /// assert!((ipark.get_sin() - 0.707).abs() < 1e-3);
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
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    /// ipark.set_angle(0.707, 0.707);  // 45° angle
    /// assert!((ipark.get_cos() - 0.707).abs() < 1e-3);
    /// ```
    pub fn get_cos(&self) -> f32 {
        self.cos
    }

    /// Gets the direct axis (d) component input.
    ///
    /// The d-axis component represents the component aligned with the rotating
    /// reference frame. In motor control, this typically corresponds to the
    /// flux command from the controller.
    ///
    /// # Returns
    ///
    /// The current direct axis component input.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    /// ipark.set_inputs(15.0, 10.0, 0.0);
    /// assert_eq!(ipark.get_d(), 15.0);
    /// ```
    pub fn get_d(&self) -> f32 {
        self.d
    }

    /// Gets the quadrature axis (q) component input.
    ///
    /// The q-axis component represents the component 90° ahead of the rotating
    /// reference frame. In motor control, this typically corresponds to the
    /// torque command from the controller.
    ///
    /// # Returns
    ///
    /// The current quadrature axis component input.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    /// ipark.set_inputs(15.0, 10.0, 0.0);
    /// assert_eq!(ipark.get_q(), 10.0);
    /// ```
    pub fn get_q(&self) -> f32 {
        self.q
    }

    /// Gets the zero-sequence component input.
    ///
    /// # Returns
    ///
    /// The zero-sequence component input value.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    /// ipark.set_inputs(1.0, 1.0, 0.2);
    /// assert_eq!(ipark.get_z(), 0.2);
    /// ```
    pub fn get_z(&self) -> f32 {
        self.z
    }

    /// Performs the inverse Park transformation calculation.
    ///
    /// This method implements the mathematical inverse Park transformation,
    /// converting the stored dq0 components into αβ0 stationary reference
    /// frame quantities using the specified rotation angle.
    ///
    /// # Mathematical Implementation
    ///
    /// ```text
    /// α = d×cos(θ) - q×sin(θ)
    /// β = d×sin(θ) + q×cos(θ)
    /// 0 = z  (zero-sequence unchanged)
    /// ```
    ///
    /// # Examples
    ///
    /// ## Aligned Reference Frame (θ = 0°)
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    /// ipark.set_inputs(10.0, 5.0, 0.0);  // d = 10, q = 5
    /// ipark.set_angle(0.0, 1.0);         // θ = 0°: sin = 0, cos = 1
    /// ipark.calculate();
    ///
    /// // When θ = 0°: α = d, β = q
    /// assert_eq!(ipark.get_alpha(), 10.0);  // d maps to α
    /// assert_eq!(ipark.get_beta(), 5.0);    // q maps to β
    /// ```
    ///
    /// ## 90° Rotated Reference Frame
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    /// ipark.set_inputs(10.0, 0.0, 0.0);   // Pure d-axis
    /// ipark.set_angle(1.0, 0.0);          // θ = 90°: sin = 1, cos = 0
    /// ipark.calculate();
    ///
    /// // When θ = 90°: α = -q, β = d
    /// assert_eq!(ipark.get_alpha(), 0.0);   // -q = 0
    /// assert_eq!(ipark.get_beta(), 10.0);   // d = 10
    /// ```
    ///
    /// ## Sinusoidal Generation
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    ///
    /// // Generate 100V sine wave by rotating constant d-axis vector
    /// ipark.set_inputs(100.0, 0.0, 0.0);  // 100V d-axis, 0V q-axis
    ///
    /// // θ = 0°: expect α = 100, β = 0
    /// ipark.set_angle(0.0, 1.0);
    /// ipark.calculate();
    /// assert_eq!(ipark.get_alpha(), 100.0);
    /// assert_eq!(ipark.get_beta(), 0.0);
    ///
    /// // θ = 90°: expect α = 0, β = 100  
    /// ipark.set_angle(1.0, 0.0);
    /// ipark.calculate();
    /// assert_eq!(ipark.get_alpha(), 0.0);
    /// assert_eq!(ipark.get_beta(), 100.0);
    /// ```
    ///
    /// ## Motor Control Voltage Commands
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let mut ipark = IPark::new(0.0, 0.0);
    ///
    /// // PI controller outputs for field-oriented control
    /// let vd_command = 5.0;   // Flux voltage from Id controller
    /// let vq_command = 12.0;  // Torque voltage from Iq controller
    ///
    /// ipark.set_inputs(vd_command, vq_command, 0.0);
    ///
    /// // Rotor angle from position sensor/estimator
    /// let theta_electrical = std::f32::consts::PI / 3.0;  // 60°
    /// ipark.set_angle(theta_electrical.sin(), theta_electrical.cos());
    /// ipark.calculate();
    ///
    /// // These αβ voltages feed into space vector PWM
    /// let v_alpha_ref = ipark.get_alpha();
    /// let v_beta_ref = ipark.get_beta();
    ///
    /// // Verify transformation preserves magnitude
    /// let dq_magnitude = (vd_command * vd_command + vq_command * vq_command).sqrt();
    /// let ab_magnitude = (v_alpha_ref * v_alpha_ref + v_beta_ref * v_beta_ref).sqrt();
    /// assert!((dq_magnitude - ab_magnitude).abs() < 1e-6);
    /// ```
    pub fn calculate(&mut self) {
        // Inverse Park transformation: dq → αβ
        self.alpha = self.d * self.cos - self.q * self.sin;
        self.beta = self.q * self.cos + self.d * self.sin;
        // Note: zero-sequence component could be passed through as:
        // self.zero = self.z;
    }
}

impl Default for IPark {
    /// Creates an inverse Park transformer with all values set to zero.
    ///
    /// The rotation angle is initialized to θ = 0° (cos = 1.0, sin = 0.0).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::transform::ipark::IPark;
    ///
    /// let ipark = IPark::default();
    /// assert_eq!(ipark.get_d(), 0.0);
    /// assert_eq!(ipark.get_q(), 0.0);
    /// assert_eq!(ipark.get_cos(), 1.0);   // θ = 0°
    /// assert_eq!(ipark.get_sin(), 0.0);
    /// ```
    fn default() -> Self {
        Self::new(0.0, 0.0)
    }
}
