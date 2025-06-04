//! # Perturb and Observe (P&O) MPPT Algorithm
//!
//! This module implements the classic Perturb and Observe Maximum Power Point Tracking
//! algorithm commonly used in photovoltaic systems. The P&O algorithm is based on the
//! hill-climbing principle where the operating point is perturbed and the resulting
//! power change is observed to determine the direction toward the maximum power point.
//!
//! ## Theory
//!
//! The P&O algorithm operates on the principle that:
//! - At the MPP: dP/dV = 0 (slope of P-V curve is zero)
//! - Left of MPP: dP/dV > 0 (increasing voltage increases power)
//! - Right of MPP: dP/dV < 0 (increasing voltage decreases power)
//!
//! ## Algorithm Logic
//!
//! ```text
//! 1. Measure current PV voltage and current
//! 2. Calculate current power: P(k) = V(k) × I(k)
//! 3. Compare with previous power: ΔP = P(k) - P(k-1)
//! 4. Determine perturbation direction:
//!    - If ΔP > 0: Continue in same direction
//!    - If ΔP < 0: Reverse direction
//! 5. Apply voltage perturbation: V_ref = V_ref ± step_size
//! 6. Update previous values for next iteration
//! ```
//!
//! ## Advantages
//!
//! - **Simplicity**: Easy to implement and understand
//! - **Low computational cost**: Minimal processing requirements
//! - **Hardware independence**: Works with any converter topology
//! - **No PV array parameters required**: Model-free operation
//!
//! ## Limitations
//!
//! - **Steady-state oscillation**: Continuously oscillates around MPP
//! - **Poor dynamic response**: Can be confused by rapid irradiance changes
//! - **Drift under uniform irradiance**: May drift away from true MPP
//! - **Step size trade-off**: Large steps = fast tracking but more oscillation
//!
//! ## Applications
//!
//! - Grid-tied solar inverters
//! - Battery charging systems  
//! - Standalone PV systems
//! - Water pumping systems
//! - Small-scale solar installations

/// MPPT action enumeration for internal state tracking.
///
/// This enum represents the direction of the next voltage perturbation
/// based on the observed power change from the previous iteration.
#[derive(Debug, Clone, Copy)]
enum VMPPAction {
    /// Increase the reference voltage
    INCREMENT,
    /// Decrease the reference voltage  
    DECREMENT,
}

/// Perturb and Observe MPPT controller implementation.
///
/// This struct maintains all the state variables and parameters required for
/// the P&O MPPT algorithm operation, including measurement storage, power
/// calculation, and voltage reference generation.
///
/// # Examples
///
/// ## Basic Usage
///
/// ```rust
/// use libpower::mppt::perturb_and_observe::MPPT;
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
/// ## Complete System Integration
///
/// ```rust
/// use libpower::mppt::perturb_and_observe::MPPT;
///
/// let mut mppt = MPPT::new();
/// mppt.set_step_size(0.05);       // Small steps for fine tracking
/// mppt.set_mppt_v_out_max(24.0);  // Battery voltage limit
/// mppt.set_mppt_v_out_min(10.0);  // Minimum operating voltage
///
/// // Simulate several control cycles
/// let measurements = [
///     (4.8, 17.2),  // (Current, Voltage) measurements
///     (4.9, 17.4),  // Power increasing: 82.56W → 85.26W
///     (5.0, 17.3),  // Power slightly decreasing: 85.26W → 86.5W
///     (5.1, 17.1),  // Continue tracking...
/// ];
///
/// for (i_pv, v_pv) in measurements.iter() {
///     mppt.calculate(*i_pv, *v_pv);
///     let v_ref = mppt.get_mppt_v_out();
///     let power = mppt.get_pv_power();
///     println!("Cycle: P={:.1}W, V_ref={:.2}V", power, v_ref);
/// }
/// ```
pub struct MPPT {
    /// Current PV current measurement (A)
    pv_i: f32,
    /// Current PV voltage measurement (V)
    pv_v: f32,
    /// Previous PV voltage measurement (V)
    pv_v_prev: f32,
    /// Current PV power calculation (W)
    pv_power: f32,
    /// Previous PV power calculation (W)
    pv_power_prev: f32,
    /// Power change from previous iteration (W)
    delta_pv_power: f32,
    /// Minimum power change threshold (W) - currently unused
    delta_p_min: f32,
    /// Next MPPT action (INCREMENT or DECREMENT)
    mppt_v_out_action: VMPPAction,
    /// Maximum voltage reference limit (V)
    mppt_v_out_max: f32,
    /// Minimum voltage reference limit (V)
    mppt_v_out_min: f32,
    /// Voltage step size for perturbation (V)
    step_size: f32,
    /// Current voltage reference output (V)
    mppt_v_out: f32,
    /// MPPT enable flag (currently unused)
    mppt_enable: bool,
    /// First calculation flag to skip initial comparison
    mppt_first: bool,
}

impl MPPT {
    /// Creates a new P&O MPPT controller with default parameters.
    ///
    /// All parameters are initialized to zero and must be configured using
    /// setter methods before operation.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::perturb_and_observe::MPPT;
    ///
    /// let mppt = MPPT::new();
    /// assert_eq!(mppt.get_mppt_v_out(), 0.0);
    /// assert_eq!(mppt.get_pv_power(), 0.0);
    /// ```
    pub fn new() -> MPPT {
        MPPT {
            pv_i: 0.0,
            pv_v: 0.0,
            pv_v_prev: 0.0,
            pv_power: 0.0,
            pv_power_prev: 0.0,
            delta_pv_power: 0.0,
            delta_p_min: 0.0,
            mppt_v_out_action: VMPPAction::INCREMENT,
            mppt_v_out_max: 0.0,
            mppt_v_out_min: 0.0,
            step_size: 0.0,
            mppt_v_out: 0.0,
            mppt_enable: true,
            mppt_first: true,
        }
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
    /// use libpower::mppt::perturb_and_observe::MPPT;
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

    /// Gets the current PV current measurement.
    ///
    /// # Returns
    ///
    /// The most recent PV current measurement in Amperes.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::perturb_and_observe::MPPT;
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
    /// use libpower::mppt::perturb_and_observe::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.calculate(5.0, 18.5);
    /// assert_eq!(mppt.get_pv_v(), 18.5);
    /// ```
    pub fn get_pv_v(&self) -> f32 {
        self.pv_v
    }

    /// Gets the current PV power calculation.
    ///
    /// Power is calculated as P = V × I using the most recent measurements.
    ///
    /// # Returns
    ///
    /// The calculated PV power in Watts.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::perturb_and_observe::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.calculate(5.0, 18.0);
    /// assert_eq!(mppt.get_pv_power(), 90.0); // 5.0A × 18.0V = 90.0W
    /// ```
    pub fn get_pv_power(&self) -> f32 {
        self.pv_power
    }

    /// Sets the maximum voltage reference limit.
    ///
    /// This limit prevents the MPPT algorithm from commanding voltages
    /// that could damage the system or exceed safe operating conditions.
    ///
    /// # Parameters
    ///
    /// * `value` - Maximum voltage reference in Volts
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::perturb_and_observe::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.set_mppt_v_out_max(30.0);  // 30V maximum for safety
    /// ```
    pub fn set_mppt_v_out_max(&mut self, value: f32) {
        self.mppt_v_out_max = value;
    }

    /// Sets the minimum voltage reference limit.
    ///
    /// This limit prevents the MPPT algorithm from commanding voltages
    /// below the minimum required for proper converter operation.
    ///
    /// # Parameters
    ///
    /// * `value` - Minimum voltage reference in Volts
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::perturb_and_observe::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.set_mppt_v_out_min(12.0);  // 12V minimum for battery charging
    /// ```
    pub fn set_mppt_v_out_min(&mut self, value: f32) {
        self.mppt_v_out_min = value;
    }

    /// Sets the voltage perturbation step size.
    ///
    /// The step size determines how much the voltage reference is adjusted
    /// at each MPPT iteration. Larger steps provide faster tracking but more
    /// oscillation around the MPP.
    ///
    /// # Parameters
    ///
    /// * `value` - Step size in Volts (typical range: 0.01V to 1.0V)
    ///
    /// # Design Guidelines
    ///
    /// - Small steps (0.01-0.1V): More accurate but slower tracking
    /// - Large steps (0.1-1.0V): Faster tracking but more oscillation
    /// - Consider PV array characteristics and converter response time
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::mppt::perturb_and_observe::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.set_step_size(0.1);  // 0.1V steps for moderate tracking speed
    /// ```
    pub fn set_step_size(&mut self, value: f32) {
        self.step_size = value;
    }

    /// Executes one iteration of the P&O MPPT algorithm.
    ///
    /// This is the main method that implements the perturb and observe logic.
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
    /// 2. Calculate current power: P = V × I
    /// 3. Compare with previous power: ΔP = P(k) - P(k-1)
    /// 4. Determine voltage perturbation direction based on ΔP and ΔV signs
    /// 5. Apply voltage step and limit to configured bounds
    /// 6. Update previous values for next iteration
    ///
    /// # Examples
    ///
    /// ## Basic Operation
    ///
    /// ```rust
    /// use libpower::mppt::perturb_and_observe::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.set_step_size(0.1);
    /// mppt.set_mppt_v_out_max(25.0);
    /// mppt.set_mppt_v_out_min(10.0);
    ///
    /// // Call periodically with sensor measurements
    /// mppt.calculate(5.0, 18.0);  // First call - just stores values
    /// mppt.calculate(5.1, 18.2);  // Second call - starts tracking
    ///
    /// let v_ref = mppt.get_mppt_v_out();  // Use this for converter control
    /// ```
    ///
    /// ## Simulated Tracking
    ///
    /// ```rust
    /// use libpower::mppt::perturb_and_observe::MPPT;
    ///
    /// let mut mppt = MPPT::new();
    /// mppt.set_step_size(0.05);
    /// mppt.set_mppt_v_out_max(20.0);
    /// mppt.set_mppt_v_out_min(15.0);
    ///
    /// // Simulate approaching MPP
    /// let test_points = [
    ///     (4.0, 16.0),  // 64W - below MPP
    ///     (4.5, 17.0),  // 76.5W - approaching MPP  
    ///     (5.0, 17.5),  // 87.5W - closer to MPP
    ///     (4.8, 17.8),  // 85.44W - past MPP, power decreasing
    /// ];
    ///
    /// for (current, voltage) in test_points.iter() {
    ///     mppt.calculate(*current, *voltage);
    ///     println!("Power: {:.1}W, V_ref: {:.2}V",
    ///              mppt.get_pv_power(), mppt.get_mppt_v_out());
    /// }
    /// ```
    pub fn calculate(&mut self, pv_i: f32, pv_v: f32) {
        if self.mppt_first {
            // Store current measurements for both old and current values on first call
            self.pv_i = pv_i;
            self.pv_v = pv_v;
            self.pv_power = pv_i * pv_v;
            self.pv_v_prev = pv_v;
            self.pv_power_prev = self.pv_power;
            self.mppt_first = false;
        } else {
            self.pv_i = pv_i;
            self.pv_v = pv_v;
            self.pv_power = self.pv_i * self.pv_v;

            // Calculate power change
            let delta_pv_power = self.pv_power - self.pv_power_prev;
            if delta_pv_power > self.delta_p_min {
                // Determine whether to INCREMENT or DECREMENT
                if self.pv_v > self.pv_v_prev {
                    self.mppt_v_out_action = VMPPAction::INCREMENT;
                } else {
                    self.mppt_v_out_action = VMPPAction::DECREMENT;
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
            }
            // Save the previous values
            self.pv_v_prev = self.pv_v;
            self.pv_power_prev = self.pv_power;
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
