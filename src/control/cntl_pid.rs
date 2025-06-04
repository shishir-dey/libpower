//! # Proportional-Integral-Derivative (PID) Controller
//!
//! This module implements a discrete-time PID controller suitable for real-time control
//! applications. The implementation includes proper derivative term calculation and
//! handles time-varying sampling periods.
//!
//! ## Theory
//!
//! The PID controller implements the continuous-time control law:
//! ```text
//! u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de(t)/dt
//! ```
//!
//! Where:
//! - `Kp` is the proportional gain
//! - `Ki` is the integral gain  
//! - `Kd` is the derivative gain
//! - `e(t)` is the error signal
//!
//! ## Discrete Implementation
//!
//! The discrete-time implementation uses:
//! ```text
//! P(k) = Kp * e(k)
//! I(k) = I(k-1) + Ki * e(k) * dt
//! D(k) = Kd * (y(k) - y(k-1)) / dt  // Derivative on measurement
//! u(k) = P(k) + I(k) + D(k)
//! ```
//!
//! ## Applications
//!
//! - General-purpose control systems
//! - Motor position control  
//! - Temperature regulation
//! - Process control systems
//! - Systems requiring derivative action for improved transient response

/// Proportional-Integral-Derivative (PID) controller.
///
/// This controller provides comprehensive control action including proportional,
/// integral, and derivative terms. The derivative term is calculated on the
/// measurement (not the error) to avoid derivative kick from setpoint changes.
///
/// # Examples
///
/// ## Basic PID Controller
///
/// ```rust
/// use libpower::control::cntl_pid::ControllerPID;
///
/// // Create PID controller with gains
/// let mut pid = ControllerPID::new(2.0, 0.1, 0.05); // Kp=2.0, Ki=0.1, Kd=0.05
///
/// // Control loop execution
/// let setpoint = 10.0;
/// let measurement = 8.0;
/// let time = 1.0; // Time in seconds
///
/// let output = pid.update(setpoint, measurement, time);
/// assert!(!output.is_nan());
/// ```
///
/// ## Proportional-Only Control
///
/// ```rust
/// use libpower::control::cntl_pid::ControllerPID;
///
/// let mut pid = ControllerPID::new(2.0, 0.0, 0.0); // P-only controller
/// let setpoint = 10.0;
/// let current_position = 8.0;
/// let current_time = 1.0;
///
/// let output = pid.update(setpoint, current_position, current_time);
///
/// // P-only: output = Kp * error
/// let error = setpoint - current_position;
/// let expected_output = 2.0 * error;
/// assert!((output - expected_output).abs() < 1e-6);
/// ```
///
/// ## Reset Controller State
///
/// ```rust
/// use libpower::control::cntl_pid::ControllerPID;
///
/// let mut pid = ControllerPID::new(1.0, 0.1, 0.01);
///
/// // Run controller to accumulate state
/// pid.update(10.0, 8.0, 1.0);
/// pid.update(10.0, 9.0, 2.0);
///
/// // Reset all state variables
/// pid.reset();
/// assert_eq!(pid.cumulative_error(), 0.0);
/// assert_eq!(pid.last_position(), 0.0);
/// ```
pub struct ControllerPID {
    /// Proportional gain (Kp)
    kp: f32,
    /// Integral gain (Ki)
    ki: f32,
    /// Derivative gain (Kd)
    kd: f32,
    /// Previous measurement value for derivative calculation
    last_position: f32,
    /// Previous time stamp for delta time calculation
    previous_time: f32,
    /// Current time stamp
    current_time: f32,
    /// Flag to skip derivative calculation on first update
    first_pass: bool,
    /// Accumulated error for integral term
    cumulative_error: f32,
}

impl ControllerPID {
    /// Creates a new PID controller with specified gains.
    ///
    /// # Parameters
    ///
    /// * `kp` - Proportional gain (typically 0.1 to 100.0)
    /// * `ki` - Integral gain (typically 0.01 to 10.0)  
    /// * `kd` - Derivative gain (typically 0.001 to 1.0)
    ///
    /// # Returns
    ///
    /// A new `ControllerPID` instance with the specified gains and reset state.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pid::ControllerPID;
    ///
    /// // Create PID controller for position control
    /// let position_controller = ControllerPID::new(5.0, 0.1, 0.02);
    ///
    /// // Create PID controller for temperature control  
    /// let temp_controller = ControllerPID::new(2.0, 0.05, 0.01);
    /// ```
    pub fn new(kp: f32, ki: f32, kd: f32) -> ControllerPID {
        ControllerPID {
            kp,
            ki,
            kd,
            last_position: 0.0,
            previous_time: 0.0,
            current_time: 0.0,
            first_pass: true,
            cumulative_error: 0.0,
        }
    }

    /// Updates the PID controller and calculates the control output.
    ///
    /// This method implements the complete PID algorithm including time-varying
    /// sample periods. The derivative term is calculated on the measurement to
    /// prevent derivative kick from setpoint changes.
    ///
    /// # Parameters
    ///
    /// * `setpoint` - Desired target value
    /// * `current_position` - Current measured value
    /// * `current_time` - Current time stamp in seconds
    ///
    /// # Returns
    ///
    /// The calculated PID controller output. Returns 0.0 if delta_time ≤ 0.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pid::ControllerPID;
    ///
    /// let mut pid = ControllerPID::new(1.0, 0.1, 0.01);
    ///
    /// // Simulate control loop at 1 Hz
    /// let mut time = 0.0;
    /// for i in 0..5 {
    ///     time += 1.0; // 1 second intervals
    ///     let setpoint = 10.0;
    ///     let measurement = 8.0 + i as f32 * 0.5; // Simulated approach to setpoint
    ///     
    ///     let output = pid.update(setpoint, measurement, time);
    ///     assert!(!output.is_nan());
    /// }
    /// ```
    pub fn update(&mut self, setpoint: f32, current_position: f32, current_time: f32) -> f32 {
        // Calculate delta_time
        let delta_time = current_time - self.previous_time;
        if delta_time <= 0.0 {
            return 0.0; // Skip updates for invalid or zero time intervals
        }

        // Calculate error
        let error = setpoint - current_position;

        // Update cumulative error (integral term)
        self.cumulative_error += error * delta_time;

        // Calculate delta_position (for derivative term)
        let delta_position = current_position - self.last_position;

        // Update internal state
        self.last_position = current_position;
        self.previous_time = current_time;

        // Compute PID terms
        let p_term = self.kp * error;
        let i_term = self.ki * self.cumulative_error;
        let d_term = if self.first_pass {
            0.0 // Skip derivative term on the first pass
        } else {
            self.kd * delta_position / delta_time
        };

        // Update first pass flag
        self.first_pass = false;

        // Compute and return output
        p_term + i_term + d_term
    }

    /// Gets the accumulated error (integral term accumulator).
    ///
    /// This value represents the time-weighted sum of all errors since the
    /// controller was created or last reset.
    ///
    /// # Returns
    ///
    /// The cumulative error value as `f32`.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pid::ControllerPID;
    ///
    /// let mut pid = ControllerPID::new(1.0, 1.0, 0.0);
    ///
    /// // Apply constant error for 3 seconds  
    /// pid.update(10.0, 8.0, 1.0); // Error = 2.0, dt = 1.0
    /// pid.update(10.0, 8.0, 2.0); // Error = 2.0, dt = 1.0
    /// pid.update(10.0, 8.0, 3.0); // Error = 2.0, dt = 1.0
    ///
    /// assert!((pid.cumulative_error() - 6.0).abs() < 1e-6); // 2*1 + 2*1 + 2*1 = 6
    /// ```
    pub fn cumulative_error(&self) -> f32 {
        self.cumulative_error
    }

    /// Gets the last measured position value.
    ///
    /// This value is used internally for derivative calculation and represents
    /// the measurement from the previous control update.
    ///
    /// # Returns
    ///
    /// The last position value as `f32`.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pid::ControllerPID;
    ///
    /// let mut pid = ControllerPID::new(1.0, 0.1, 0.01);
    ///
    /// pid.update(10.0, 8.5, 1.0);
    /// assert_eq!(pid.last_position(), 8.5);
    ///
    /// pid.update(10.0, 9.2, 2.0);  
    /// assert_eq!(pid.last_position(), 9.2);
    /// ```
    pub fn last_position(&self) -> f32 {
        self.last_position
    }

    /// Resets all internal state variables to their initial values.
    ///
    /// This method clears the integral accumulator, resets position tracking,
    /// and reinitializes the first-pass flag. Use this when starting a new
    /// control sequence or when discontinuous operation requires state reset.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::control::cntl_pid::ControllerPID;
    ///
    /// let mut pid = ControllerPID::new(1.0, 0.1, 0.01);
    ///
    /// // Accumulate some integral action
    /// pid.update(10.0, 5.0, 1.0);
    /// pid.update(10.0, 6.0, 2.0);
    /// assert!(pid.cumulative_error() > 0.0);
    ///
    /// // Reset controller state
    /// pid.reset();
    /// assert_eq!(pid.cumulative_error(), 0.0);
    /// assert_eq!(pid.last_position(), 0.0);
    /// ```
    pub fn reset(&mut self) {
        self.last_position = 0.0;
        self.previous_time = 0.0;
        self.current_time = 0.0;
        self.first_pass = true;
        self.cumulative_error = 0.0;
    }
}
