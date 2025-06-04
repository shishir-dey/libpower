//! # Proportional-Integral (PI) Controller
//!
//! This module implements a discrete-time PI controller with anti-windup protection,
//! optimized for real-time control applications in power electronics systems.
//!
//! ## Theory
//!
//! The PI controller implements the discrete-time transfer function:
//! ```text
//! C(z) = Kp + Ki * z/(z-1)
//! ```
//!
//! Where:
//! - `Kp` is the proportional gain
//! - `Ki` is the integral gain
//! - The integral term provides zero steady-state error
//! - Anti-windup prevents integrator saturation
//!
//! ## Implementation Details
//!
//! The controller uses the velocity form with anti-windup:
//! ```text
//! up(k) = Kp * e(k)                    // Proportional term
//! ui(k) = ui(k-1) + Ki * e(k) + w(k-1) // Integral term with anti-windup
//! u(k) = sat(up(k) + ui(k))            // Saturated output
//! w(k) = u(k) - (up(k) + ui(k))        // Anti-windup correction
//! ```
//!
//! ## Applications
//!
//! - DC-DC converter current control loops
//! - Voltage regulation in power supplies
//! - Motor speed control
//! - Temperature control systems
//! - Grid-tied inverter control

/// Proportional-Integral (PI) controller with anti-windup protection.
///
/// This controller provides robust control with zero steady-state error while preventing
/// integrator windup through built-in saturation handling. The implementation uses the
/// velocity form for improved numerical stability and anti-windup performance.
///
/// # Examples
///
/// ## Basic Usage
///
/// ```rust
/// use libpower::control::cntl_pi::ControllerPI;
///
/// // Create a PI controller with default parameters
/// let mut controller = ControllerPI::new();
///
/// // Set gains for a current control loop
/// controller.set_gains(0.5, 100.0);  // Kp=0.5, Ki=100.0
/// controller.set_limits(-1.0, 1.0);  // ±1.0 duty cycle limits
///
/// // Execute control loop
/// let setpoint = 5.0;     // 5A current reference
/// let measurement = 4.8;  // 4.8A measured current
/// let output = controller.calculate(setpoint, measurement);
///
/// assert!(output >= -1.0 && output <= 1.0);
/// ```
///
/// ## Creating with Specific Gains
///
/// ```rust
/// use libpower::control::cntl_pi::ControllerPI;
///
/// // Create controller with specified gains
/// let mut controller = ControllerPI::with_gains(1.0, 0.1);
/// assert_eq!(controller.get_kp(), 1.0);
/// assert_eq!(controller.get_ki(), 0.1);
/// ```
///
/// ## Reset for State Initialization
///
/// ```rust
/// use libpower::control::cntl_pi::ControllerPI;
///
/// let mut controller = ControllerPI::with_gains(1.0, 0.1);
///
/// // Run some control cycles
/// controller.calculate(1.0, 0.0);
/// assert!(controller.get_output() != 0.0);
///
/// // Reset controller state
/// controller.reset();
/// assert_eq!(controller.get_output(), 0.0);
/// assert_eq!(controller.get_integral_term(), 0.0);
/// ```
#[derive(Debug, Clone)]
pub struct ControllerPI {
    // Inputs
    /// Reference set-point value
    ref_input: f32,
    /// Feedback measurement value
    fdbk: f32,

    // Outputs
    /// Controller output value
    out: f32,

    // Parameters
    /// Proportional gain (Kp)
    kp: f32,
    /// Integral gain (Ki)
    ki: f32,
    /// Upper saturation limit
    u_max: f32,
    /// Lower saturation limit
    u_min: f32,

    // Internal state
    /// Proportional term value
    up: f32,
    /// Integral term value
    ui: f32,
    /// Pre-saturated controller output
    v1: f32,
    /// Integrator storage: ui(k-1)
    i1: f32,
    /// Saturation record: [u(k-1) - v(k-1)] for anti-windup
    w1: f32,
}

impl Default for ControllerPI {
    /// Creates a PI controller with default parameters.
    ///
    /// Default values:
    /// - Kp = 0.2
    /// - Ki = 0.1
    /// - Output limits = ±f32::MAX (no saturation)
    /// - All state variables = 0.0
    fn default() -> Self {
        ControllerPI {
            ref_input: 0.0,
            fdbk: 0.0,
            out: 0.0,
            kp: 0.2,
            ki: 0.1,
            u_max: f32::MAX,
            u_min: f32::MIN,
            up: 0.0,
            ui: 0.0,
            v1: 0.0,
            i1: 0.0,
            w1: 0.0,
        }
    }
}

impl ControllerPI {
    /// Creates a new PI controller with default parameters.
    ///
    /// This is equivalent to using `Default::default()` but provides a more explicit
    /// constructor pattern commonly used in control system implementations.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pi::ControllerPI;
    ///
    /// let controller = ControllerPI::new();
    /// assert_eq!(controller.get_kp(), 0.2);
    /// assert_eq!(controller.get_ki(), 0.1);
    /// assert_eq!(controller.get_output(), 0.0);
    /// ```
    pub fn new() -> Self {
        Default::default()
    }

    /// Gets the current proportional gain (Kp).
    ///
    /// # Returns
    ///
    /// The proportional gain value as `f32`.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pi::ControllerPI;
    ///
    /// let controller = ControllerPI::with_gains(2.5, 0.1);
    /// assert_eq!(controller.get_kp(), 2.5);
    /// ```
    pub fn get_kp(&self) -> f32 {
        self.kp
    }

    /// Gets the current integral gain (Ki).
    ///
    /// # Returns
    ///
    /// The integral gain value as `f32`.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pi::ControllerPI;
    ///
    /// let controller = ControllerPI::with_gains(1.0, 0.05);
    /// assert_eq!(controller.get_ki(), 0.05);
    /// ```
    pub fn get_ki(&self) -> f32 {
        self.ki
    }

    /// Creates a PI controller with specified gains.
    ///
    /// # Parameters
    ///
    /// * `kp` - Proportional gain (typically 0.1 to 10.0 for most applications)
    /// * `ki` - Integral gain (typically 0.01 to 1000.0 depending on sampling frequency)
    ///
    /// # Returns
    ///
    /// A new `ControllerPI` instance with the specified gains.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pi::ControllerPI;
    ///
    /// // For a current control loop at 10 kHz sampling
    /// let current_controller = ControllerPI::with_gains(0.5, 100.0);
    ///
    /// // For a voltage control loop at 1 kHz sampling  
    /// let voltage_controller = ControllerPI::with_gains(2.0, 500.0);
    /// ```
    pub fn with_gains(kp: f32, ki: f32) -> Self {
        let mut controller = Self::default();
        controller.kp = kp;
        controller.ki = ki;
        controller
    }

    /// Sets the proportional and integral gains.
    ///
    /// This method allows updating the controller gains during runtime without
    /// resetting the internal state variables.
    ///
    /// # Parameters
    ///
    /// * `kp` - New proportional gain
    /// * `ki` - New integral gain
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pi::ControllerPI;
    ///
    /// let mut controller = ControllerPI::new();
    /// controller.set_gains(1.5, 0.2);
    ///
    /// assert_eq!(controller.get_kp(), 1.5);
    /// assert_eq!(controller.get_ki(), 0.2);
    /// ```
    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.kp = kp;
        self.ki = ki;
    }

    /// Sets the output saturation limits.
    ///
    /// Saturation limits prevent the controller output from exceeding physical
    /// constraints and provide anti-windup protection for the integral term.
    ///
    /// # Parameters
    ///
    /// * `u_min` - Lower saturation limit
    /// * `u_max` - Upper saturation limit
    ///
    /// # Panics
    ///
    /// This method will not panic but `u_min` should be less than `u_max` for
    /// proper operation.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pi::ControllerPI;
    ///
    /// let mut controller = ControllerPI::new();
    ///
    /// // Set limits for PWM duty cycle (0% to 100%)
    /// controller.set_limits(0.0, 1.0);
    ///
    /// // Set limits for bipolar output (±10V)
    /// controller.set_limits(-10.0, 10.0);
    /// ```
    pub fn set_limits(&mut self, u_min: f32, u_max: f32) {
        self.u_min = u_min;
        self.u_max = u_max;
    }

    /// Calculates the controller output for given reference and feedback values.
    ///
    /// This is the main method that implements the PI control algorithm with
    /// anti-windup protection. It should be called at each control sample period.
    ///
    /// # Parameters
    ///
    /// * `ref_input` - Reference set-point value
    /// * `fdbk` - Feedback measurement value
    ///
    /// # Returns
    ///
    /// The calculated controller output, limited by the saturation bounds.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pi::ControllerPI;
    ///
    /// let mut controller = ControllerPI::with_gains(1.0, 0.1);
    /// controller.set_limits(-0.5, 0.5);
    ///
    /// // Control loop execution
    /// let output = controller.calculate(1.0, 0.0); // Error = 1.0
    /// assert!(output >= -0.5 && output <= 0.5);
    ///
    /// // Steady-state: output should drive error to zero over time
    /// for _ in 0..100 {
    ///     let current_output = controller.calculate(1.0, 1.0); // Error = 0.0
    /// }
    /// ```
    pub fn calculate(&mut self, ref_input: f32, fdbk: f32) -> f32 {
        self.ref_input = ref_input;
        self.fdbk = fdbk;

        // Calculate error
        let error = self.ref_input - self.fdbk;

        // Compute proportional term
        self.up = self.kp * error;

        // Compute integral term with anti-windup
        self.ui = self.i1 + (self.ki * error) + (self.w1);

        // Compute controller output
        self.v1 = self.up + self.ui;

        // Apply saturation limits
        self.out = self.v1.min(self.u_max).max(self.u_min);

        // Compute saturation record for anti-windup
        self.w1 = self.out - self.v1;

        // Store integral term for next iteration
        self.i1 = self.ui;

        self.out
    }

    /// Resets all internal state variables to zero.
    ///
    /// This method should be called when starting a new control sequence or
    /// when the system requires reinitialization. It clears all accumulated
    /// integral action and internal states.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pi::ControllerPI;
    ///
    /// let mut controller = ControllerPI::with_gains(1.0, 0.1);
    ///
    /// // Accumulate some integral action
    /// controller.calculate(1.0, 0.0);
    /// assert!(controller.get_integral_term() != 0.0);
    ///
    /// // Reset controller
    /// controller.reset();
    /// assert_eq!(controller.get_output(), 0.0);
    /// assert_eq!(controller.get_integral_term(), 0.0);
    /// assert_eq!(controller.get_proportional_term(), 0.0);
    /// ```
    pub fn reset(&mut self) {
        self.up = 0.0;
        self.ui = 0.0;
        self.v1 = 0.0;
        self.i1 = 0.0;
        self.w1 = 0.0;
        self.out = 0.0;
    }

    // Getters

    /// Gets the current controller output.
    ///
    /// # Returns
    ///
    /// The most recent controller output value, after saturation limiting.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pi::ControllerPI;
    ///
    /// let mut controller = ControllerPI::with_gains(1.0, 0.0); // P-only
    /// controller.calculate(1.0, 0.0); // Error = 1.0
    /// assert_eq!(controller.get_output(), 1.0);
    /// ```
    pub fn get_output(&self) -> f32 {
        self.out
    }

    /// Gets the current proportional term value.
    ///
    /// # Returns
    ///
    /// The proportional component (Kp * error) of the controller output.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pi::ControllerPI;
    ///
    /// let mut controller = ControllerPI::with_gains(2.0, 0.0);
    /// controller.calculate(1.0, 0.5); // Error = 0.5
    /// assert_eq!(controller.get_proportional_term(), 1.0); // 2.0 * 0.5
    /// ```
    pub fn get_proportional_term(&self) -> f32 {
        self.up
    }

    /// Gets the current integral term value.
    ///
    /// # Returns
    ///
    /// The integral component of the controller output.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pi::ControllerPI;
    ///
    /// let mut controller = ControllerPI::with_gains(0.0, 1.0); // I-only
    ///
    /// // Accumulate integral action
    /// controller.calculate(1.0, 0.0); // Error = 1.0
    /// controller.calculate(1.0, 0.0); // Error = 1.0
    /// controller.calculate(1.0, 0.0); // Error = 1.0
    ///
    /// assert!(controller.get_integral_term() > 2.0); // Should accumulate
    /// ```
    pub fn get_integral_term(&self) -> f32 {
        self.ui
    }
}
