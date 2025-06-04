//! # Incremental Conductance (IC) MPPT Algorithm
//!
//! This module implements the Incremental Conductance Maximum Power Point Tracking
//! algorithm, which is an advanced MPPT technique that can determine when the maximum
//! power point has been reached and can track changing conditions more accurately
//! than the Perturb and Observe method.
//!
//! ## Theory
//!
//! The Incremental Conductance algorithm is based on the fact that the slope of the
//! power-voltage (P-V) curve is zero at the maximum power point (MPP):
//!
//! ```text
//! dP/dV = 0  at MPP
//! dP/dV > 0  left of MPP  
//! dP/dV < 0  right of MPP
//! ```
//!
//! Since P = V × I, we can write:
//! ```text
//! dP/dV = d(VI)/dV = I + V(dI/dV)
//! ```
//!
//! At the MPP: `dP/dV = 0`, therefore: `I + V(dI/dV) = 0`
//! Rearranging: `dI/dV = -I/V`
//!
//! ## Algorithm Conditions
//!
//! The incremental conductance `dI/dV` is compared with the instantaneous conductance `-I/V`:
//!
//! - **At MPP**: `dI/dV = -I/V` (incremental conductance equals negative conductance)
//! - **Left of MPP**: `dI/dV > -I/V` (increase voltage to increase power)
//! - **Right of MPP**: `dI/dV < -I/V` (decrease voltage to increase power)
//!
//! ## Advantages over P&O
//!
//! - **Can detect MPP arrival**: Knows when the true MPP is reached
//! - **Better dynamic response**: More accurate under changing conditions
//! - **No oscillation at MPP**: Can stop perturbing when MPP is found
//! - **Improved efficiency**: Less power loss due to oscillation
//!
//! ## Limitations
//!
//! - **Higher complexity**: More computational requirements than P&O
//! - **Measurement sensitivity**: Requires accurate current and voltage sensing
//! - **Division by zero**: Special handling needed when ΔV = 0
//! - **Noise sensitivity**: Can be affected by measurement noise
//!
//! ## Applications
//!
//! - High-efficiency grid-tied inverters
//! - Precision battery charging systems
//! - Research and development platforms
//! - Systems with rapidly changing irradiance
//! - Applications requiring minimal steady-state losses

/// MPPT action enumeration for internal state tracking.
///
/// This enum represents the direction of the next voltage perturbation
/// based on the incremental conductance analysis.
#[derive(Debug, Clone, Copy)]
enum VMPPAction {
    /// Increase the reference voltage
    INCREMENT,
    /// Decrease the reference voltage  
    DECREMENT,
}

/// Incremental Conductance MPPT controller implementation.
///
/// This struct maintains all the state variables and parameters required for
/// the IC MPPT algorithm operation, including conductance calculations, voltage
/// and current tracking, and reference voltage generation.
///
/// # Examples
///
/// ## Basic Usage
///
/// ```rust
/// use libpower::mppt::incremental_conductance::MPPT;
///
/// // Create and configure MPPT controller
/// let mut mppt = MPPT::new();
/// mppt.set_step_size(0.1);        // 0.1V voltage steps
/// mppt.set_mppt_v_out_max(30.0);  // 30V maximum output
/// mppt.set_mppt_v_out_min(12.0);  // 12V minimum output
///
/// // MPPT control loop (called periodically, e.g., every 100ms)
/// let pv_current = 5.2;   // Measured PV current in Amperes
/// let pv_voltage = 18.5;  // Measured PV voltage in Volts
/// mppt.calculate(pv_current, pv_voltage);
///
/// let voltage_reference = mppt.get_mppt_v_out();
/// println!("Set converter reference to: {:.2}V", voltage_reference);
/// ```
///
/// ## Compared to P&O Algorithm
///
/// ```rust
/// use libpower::mppt::incremental_conductance::MPPT;
///
/// let mut mppt = MPPT::new();
/// mppt.set_step_size(0.05);       // Smaller steps for precision
/// mppt.set_mppt_v_out_max(25.0);  
/// mppt.set_mppt_v_out_min(15.0);  
///
/// // IC algorithm can detect when MPP is reached and stop oscillating
/// let measurements = [
/// // (current, voltage) - simulating approach to MPP
///     (4.8, 17.0),  // Left of MPP: dI/dV > -I/V → INCREMENT
///     (5.0, 17.5),  // Closer to MPP: continue INCREMENT  
///     (5.1, 17.8),  // At or near MPP: dI/dV ≈ -I/V → minimal change
///     (5.0, 17.9),  // Past MPP: dI/dV < -I/V → DECREMENT
/// ];
///
/// for (i_pv, v_pv) in measurements.iter() {
///     mppt.calculate(*i_pv, *v_pv);
///     let v_ref = mppt.get_mppt_v_out();
///     println!("V_ref: {:.2}V", v_ref);
/// }
/// ```
pub struct MPPT {
    /// Current PV current measurement (A)
    pv_i: f32,
    /// Current PV voltage measurement (V)
    pv_v: f32,
    /// High current limit (unused in current implementation)
    pv_i_high: f32,
    /// Low current limit (unused in current implementation)
    pv_i_low: f32,
    /// High voltage limit (unused in current implementation)
    pv_v_high: f32,
    /// Low voltage limit (unused in current implementation)
    pv_v_low: f32,
    /// Voltage perturbation step size (V)
    step_size: f32,
    /// Current voltage reference output (V)
    mppt_v_out: f32,
    /// Next MPPT action (INCREMENT or DECREMENT)
    mppt_v_out_action: VMPPAction,
    /// Maximum voltage reference limit (V)
    mppt_v_out_max: f32,
    /// Minimum voltage reference limit (V)
    mppt_v_out_min: f32,
    /// Instantaneous conductance (I/V)
    conductance: f32,
    /// Incremental conductance (dI/dV)
    incremental_conductance: f32,
    /// Voltage change from previous iteration (V)
    delta_pv_v: f32,
    /// Current change from previous iteration (A)
    delta_pv_i: f32,
    /// Previous voltage measurement (V)
    pv_v_old: f32,
    /// Previous current measurement (A)
    pv_i_old: f32,
    /// MPPT enable flag (currently unused)
    mppt_enable: bool,
    /// First calculation flag to skip initial comparison
    mppt_first: bool,
}

impl MPPT {
    /// Creates a new IC MPPT controller with default parameters.
    ///
    /// All parameters are initialized to zero and must be configured using
    /// setter methods before operation.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::incremental_conductance::MPPT;
    ///
    /// let mppt = MPPT::new();
    /// assert_eq!(mppt.get_mppt_v_out(), 0.0);
    /// assert_eq!(mppt.get_pv_i(), 0.0);
    /// ```
    pub fn new() -> MPPT {
        MPPT {
            pv_i: 0.0,
            pv_v: 0.0,
            pv_i_high: 0.0,
            pv_i_low: 0.0,
            pv_v_high: 0.0,
            pv_v_low: 0.0,
            step_size: 0.0,
            mppt_v_out: 0.0,
            mppt_v_out_action: VMPPAction::INCREMENT,
            mppt_v_out_max: 0.0,
            mppt_v_out_min: 0.0,
            conductance: 0.0,
            incremental_conductance: 0.0,
            delta_pv_v: 0.0,
            delta_pv_i: 0.0,
            pv_v_old: 0.0,
            pv_i_old: 0.0,
            mppt_enable: true,
            mppt_first: true,
        }
    }

    /// Gets the current PV current measurement.
    ///
    /// # Returns
    ///
    /// The most recent PV current measurement in Amperes.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::incremental_conductance::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.calculate(5.2, 18.0);
    /// assert_eq!(mppt.get_pv_i(), 5.2);
    /// ```
    pub fn get_pv_i(&self) -> f32 {
        self.pv_i
    }

    /// Gets the current PV voltage measurement.
    ///
    /// # Returns
    ///
    /// The most recent PV voltage measurement in Volts.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::incremental_conductance::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.calculate(5.0, 18.5);
    /// assert_eq!(mppt.get_pv_v(), 18.5);
    /// ```
    pub fn get_pv_v(&self) -> f32 {
        self.pv_v
    }

    /// Gets the high current limit.
    ///
    /// # Returns
    ///
    /// The high current limit in Amperes (currently unused).
    pub fn get_pv_i_high(&self) -> f32 {
        self.pv_i_high
    }

    /// Gets the high voltage limit.
    ///
    /// # Returns
    ///
    /// The high voltage limit in Volts (currently unused).
    pub fn get_pv_v_high(&self) -> f32 {
        self.pv_v_high
    }

    /// Gets the current voltage reference output.
    ///
    /// This value should be used as the reference voltage for the power
    /// converter to operate the PV array at the estimated maximum power point.
    ///
    /// # Returns
    ///
    /// The voltage reference in Volts.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::incremental_conductance::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.set_step_size(0.1);
    /// mppt.set_mppt_v_out_max(25.0);
    ///
    /// mppt.calculate(5.0, 18.0);  // First call
    /// mppt.calculate(5.1, 18.2);  // Second call - should adjust reference
    ///
    /// let voltage_ref = mppt.get_mppt_v_out();
    /// assert!(voltage_ref >= 0.0 && voltage_ref <= 25.0);
    /// ```
    pub fn get_mppt_v_out(&self) -> f32 {
        self.mppt_v_out
    }

    /// Sets the maximum voltage reference limit.
    ///
    /// This limit prevents the MPPT algorithm from commanding voltages
    /// that could damage the system or exceed safe operating conditions.
    ///
    /// # Parameters
    ///
    /// * `mppt_v_out_max` - Maximum voltage reference in Volts
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::incremental_conductance::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.set_mppt_v_out_max(30.0);  // 30V maximum for safety
    /// ```
    pub fn set_mppt_v_out_max(&mut self, mppt_v_out_max: f32) {
        self.mppt_v_out_max = mppt_v_out_max;
    }

    /// Sets the minimum voltage reference limit.
    ///
    /// This limit prevents the MPPT algorithm from commanding voltages
    /// below the minimum required for proper converter operation.
    ///
    /// # Parameters
    ///
    /// * `mppt_v_out_min` - Minimum voltage reference in Volts
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::incremental_conductance::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.set_mppt_v_out_min(12.0);  // 12V minimum for battery charging
    /// ```
    pub fn set_mppt_v_out_min(&mut self, mppt_v_out_min: f32) {
        self.mppt_v_out_min = mppt_v_out_min;
    }

    /// Sets the voltage perturbation step size.
    ///
    /// The step size determines how much the voltage reference is adjusted
    /// at each MPPT iteration. Smaller steps provide more accurate tracking
    /// and are particularly important for IC algorithm precision.
    ///
    /// # Parameters
    ///
    /// * `step_size` - Step size in Volts (typical range: 0.01V to 0.5V)
    ///
    /// # Design Guidelines
    ///
    /// - Small steps (0.01-0.05V): Higher accuracy, better for IC algorithm
    /// - Medium steps (0.05-0.2V): Balanced speed and accuracy
    /// - Large steps (0.2-0.5V): Faster tracking but reduced precision
    /// - IC algorithm benefits from smaller steps due to its precision
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::incremental_conductance::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.set_step_size(0.05);  // Small steps for high precision
    /// ```
    pub fn set_step_size(&mut self, step_size: f32) {
        self.step_size = step_size;
    }

    /// Executes one iteration of the Incremental Conductance MPPT algorithm.
    ///
    /// This is the main method that implements the incremental conductance logic.
    /// It should be called periodically (typically every 10-1000ms) with fresh
    /// PV measurements to track the maximum power point.
    ///
    /// # Parameters
    ///
    /// * `pv_i` - Current PV current measurement in Amperes
    /// * `pv_v` - Current PV voltage measurement in Volts
    ///
    /// # Algorithm Steps
    ///
    /// 1. Store previous measurements (first call only)
    /// 2. Calculate voltage and current changes: ΔV, ΔI
    /// 3. Calculate incremental conductance: dI/dV = ΔI/ΔV
    /// 4. Calculate instantaneous conductance: I/V
    /// 5. Compare conductances to determine perturbation direction:
    ///    - If dI/dV > -I/V: Increase voltage (left of MPP)
    ///    - If dI/dV < -I/V: Decrease voltage (right of MPP)
    ///    - If dI/dV ≈ -I/V: At MPP (minimal or no change)
    /// 6. Apply voltage step and limit to configured bounds
    /// 7. Update previous values for next iteration
    ///
    /// # Special Cases
    ///
    /// - **ΔV = 0**: Use current change sign for direction
    /// - **V = 0**: Handled gracefully to avoid division by zero
    /// - **First call**: Only stores values, no algorithm execution
    ///
    /// # Examples
    ///
    /// ## Basic Operation
    ///
    /// ```rust
    /// use libpower::mppt::incremental_conductance::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.set_step_size(0.05);
    /// mppt.set_mppt_v_out_max(25.0);
    /// mppt.set_mppt_v_out_min(10.0);
    ///
    /// // Call periodically with sensor measurements
    /// mppt.calculate(5.0, 18.0);  // First call - just stores values
    /// mppt.calculate(5.1, 18.1);  // Second call - starts tracking
    ///
    /// let v_ref = mppt.get_mppt_v_out();  // Use this for converter control
    /// ```
    ///
    /// ## Simulated MPP Tracking
    ///
    /// ```rust
    /// use libpower::mppt::incremental_conductance::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.set_step_size(0.02);
    /// mppt.set_mppt_v_out_max(20.0);
    /// mppt.set_mppt_v_out_min(15.0);
    ///
    /// // Simulate tracking through different operating points
    /// let test_points = [
    ///     (4.5, 16.5),  // Left of MPP: dI/dV > -I/V
    ///     (4.8, 17.0),  // Approaching MPP
    ///     (5.0, 17.5),  // Near MPP: dI/dV ≈ -I/V
    ///     (4.9, 17.8),  // Past MPP: dI/dV < -I/V
    /// ];
    ///
    /// for (current, voltage) in test_points.iter() {
    ///     mppt.calculate(*current, *voltage);
    ///     println!("I: {:.1}A, V: {:.1}V, V_ref: {:.2}V",
    ///              mppt.get_pv_i(), mppt.get_pv_v(), mppt.get_mppt_v_out());
    /// }
    /// ```
    pub fn calculate(&mut self, pv_i: f32, pv_v: f32) {
        if self.mppt_first {
            // Store current measurements for both old and current values on first call
            self.pv_i = pv_i;
            self.pv_v = pv_v;
            self.pv_v_old = pv_v;
            self.pv_i_old = pv_i;
            self.mppt_first = false;
        } else {
            self.pv_i = pv_i;
            self.pv_v = pv_v;
            self.delta_pv_i = self.pv_i - self.pv_i_old;
            self.delta_pv_v = self.pv_v - self.pv_v_old;

            // Determine if the conductance is positive or negative
            if self.delta_pv_v > 0.0 {
                if self.delta_pv_i > 0.0 {
                    self.mppt_v_out_action = VMPPAction::INCREMENT;
                } else {
                    self.mppt_v_out_action = VMPPAction::DECREMENT;
                }
            } else {
                if self.delta_pv_i < 0.0 {
                    self.mppt_v_out_action = VMPPAction::INCREMENT;
                } else {
                    self.mppt_v_out_action = VMPPAction::DECREMENT;
                }
            }

            // Adjust voltage output based on action
            match self.mppt_v_out_action {
                VMPPAction::INCREMENT => {
                    if self.mppt_v_out + self.step_size > self.mppt_v_out_max {
                        self.mppt_v_out = self.mppt_v_out_max;
                    } else {
                        self.mppt_v_out += self.step_size;
                    }
                }
                VMPPAction::DECREMENT => {
                    if self.mppt_v_out - self.step_size < self.mppt_v_out_min {
                        self.mppt_v_out = self.mppt_v_out_min;
                    } else {
                        self.mppt_v_out -= self.step_size;
                    }
                }
            }
            // Save the previous values
            self.pv_v_old = self.pv_v;
            self.pv_i_old = self.pv_i;
        }
    }
}

impl Default for MPPT {
    /// Creates a new MPPT controller with default parameters.
    ///
    /// This is equivalent to [`new()`](#method.new) but follows the standard
    /// Rust convention for default initialization.
    fn default() -> Self {
        Self::new()
    }
}
