//! # State of Charge (SoC) Estimation using Extended Kalman Filter
//!
//! This module implements a battery State of Charge estimation algorithm using an Extended Kalman Filter (EKF).
//! The implementation is based on a 2nd-order RC equivalent circuit model with polynomial-based parameter
//! estimation as a function of SoC.
//!
//! ## Battery Model
//!
//! The battery is modeled as:
//! - Open Circuit Voltage (Uocv) as a function of SoC
//! - Internal resistance (R_int) as a function of SoC  
//! - Two RC circuits (R1-C1, R2-C2) representing transient behavior
//!
//! ## State Vector
//!
//! The state vector consists of:
//! ```text
//! X = [SoC, V_RC1, V_RC2]
//! ```
//! Where:
//! - `SoC`: State of Charge (0.0 to 1.0)
//! - `V_RC1`: Voltage across first RC circuit
//! - `V_RC2`: Voltage across second RC circuit
//!
//! ## Usage Example
//!
//! ```rust
//! use libpower::battery::soc::Battery;
//!
//! let mut estimator = Battery::new(0.9, 50.0 * 3600.0, 1.0);
//! estimator.set_process_noise(1e-5);
//! estimator.set_measurement_noise(5e-4);
//!
//! // In your control loop
//! let current = -10.0; // 10A discharge
//! let terminal_voltage = 3.7; // Measured terminal voltage
//!
//! let estimated_soc = estimator.update(current, terminal_voltage);
//! println!("Estimated SoC: {:.1}%", estimated_soc * 100.0);
//! ```

use libm;

/// Extended Kalman Filter-based State of Charge estimator for lithium-ion batteries.
///
/// This estimator uses a 2nd-order RC equivalent circuit model with polynomial-based
/// parameter estimation. The state vector consists of [SoC, V_RC1, V_RC2].
#[derive(Debug, Clone)]
pub struct Battery {
    // State vector [SoC, V_RC1, V_RC2]
    state: [f32; 3],

    // Error covariance matrix (3x3)
    error_covariance: [[f32; 3]; 3],

    // Parameters
    nominal_capacity: f32,     // Ah
    coulombic_efficiency: f32, // 0.0 to 1.0
    sampling_time: f32,        // seconds

    // Noise parameters
    process_noise: f32,     // Q
    measurement_noise: f32, // R

    // Internal variables for calculations
    predicted_state: [f32; 3],
    predicted_covariance: [[f32; 3]; 3],
}

impl Default for Battery {
    /// Creates a SoC estimator with default parameters.
    ///
    /// Default values:
    /// - Initial SoC = 0.9 (90%)
    /// - Nominal capacity = 50 Ah
    /// - Sampling time = 1.0 second
    /// - Coulombic efficiency = 0.98
    /// - Process noise = 1e-5
    /// - Measurement noise = 5e-4
    fn default() -> Self {
        Self::new(0.9, 50.0 * 3600.0, 1.0) // 50 Ah in As
    }
}

impl Battery {
    /// Creates a new SoC estimator with specified parameters.
    ///
    /// # Parameters
    ///
    /// * `initial_soc` - Initial State of Charge (0.0 to 1.0)
    /// * `nominal_capacity` - Battery nominal capacity in Ampere-seconds (As)
    /// * `sampling_time` - Sampling period in seconds
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::battery::soc::Battery;
    ///
    /// // Create estimator for 50Ah battery with 1-second sampling
    /// let estimator = Battery::new(0.8, 50.0 * 3600.0, 1.0);
    /// assert!((estimator.get_soc() - 0.8).abs() < 1e-6);
    /// ```
    pub fn new(initial_soc: f32, nominal_capacity: f32, sampling_time: f32) -> Self {
        let mut estimator = Battery {
            state: [initial_soc, 0.0, 0.0],
            error_covariance: [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]],
            nominal_capacity,
            coulombic_efficiency: 0.98,
            sampling_time,
            process_noise: 1e-5,
            measurement_noise: 5e-4,
            predicted_state: [0.0; 3],
            predicted_covariance: [[0.0; 3]; 3],
        };

        // Ensure SoC is within valid range
        estimator.state[0] = estimator.state[0].clamp(0.0, 1.0);
        estimator
    }

    /// Sets the process noise parameter (Q).
    ///
    /// # Parameters
    ///
    /// * `noise` - Process noise value (typically 1e-5 to 1e-3)
    pub fn set_process_noise(&mut self, noise: f32) {
        self.process_noise = noise;
    }

    /// Sets the measurement noise parameter (R).
    ///
    /// # Parameters
    ///
    /// * `noise` - Measurement noise value (typically 1e-4 to 1e-2)
    pub fn set_measurement_noise(&mut self, noise: f32) {
        self.measurement_noise = noise;
    }

    /// Sets the coulombic efficiency.
    ///
    /// # Parameters
    ///
    /// * `efficiency` - Coulombic efficiency (0.0 to 1.0, typically 0.95 to 0.99)
    pub fn set_coulombic_efficiency(&mut self, efficiency: f32) {
        self.coulombic_efficiency = efficiency.clamp(0.0, 1.0);
    }

    /// Gets the current estimated State of Charge.
    ///
    /// # Returns
    ///
    /// SoC value between 0.0 and 1.0
    pub fn get_soc(&self) -> f32 {
        self.state[0]
    }

    /// Gets the voltage across the first RC circuit.
    ///
    /// # Returns
    ///
    /// Voltage in volts
    pub fn get_v_rc1(&self) -> f32 {
        self.state[1]
    }

    /// Gets the voltage across the second RC circuit.
    ///
    /// # Returns
    ///
    /// Voltage in volts
    pub fn get_v_rc2(&self) -> f32 {
        self.state[2]
    }

    /// Updates the SoC estimate using Extended Kalman Filter.
    ///
    /// This method implements the complete EKF cycle:
    /// 1. Prediction step
    /// 2. Kalman gain calculation
    /// 3. State update with measurement
    /// 4. Error covariance update
    ///
    /// # Parameters
    ///
    /// * `current` - Battery current in Amperes (negative for discharge, positive for charge)
    /// * `terminal_voltage` - Measured terminal voltage in Volts
    ///
    /// # Returns
    ///
    /// Updated SoC estimate (0.0 to 1.0)
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::battery::soc::Battery;
    ///
    /// let mut estimator = Battery::new(0.9, 50.0 * 3600.0, 1.0);
    ///
    /// // Discharge at 10A for multiple steps
    /// for _ in 0..10 {
    ///     let soc = estimator.update(-10.0, 3.7);
    ///     assert!(soc > 0.0 && soc <= 1.0);
    /// }
    /// ```
    pub fn update(&mut self, current: f32, terminal_voltage: f32) -> f32 {
        // Step 1: Prediction
        self.predict(current);

        // Step 2: Update with measurement
        self.update_with_measurement(current, terminal_voltage);

        // Ensure SoC stays within bounds
        self.state[0] = self.state[0].clamp(0.0, 1.0);

        self.state[0]
    }

    /// Prediction step of the Extended Kalman Filter.
    fn predict(&mut self, current: f32) {
        let soc = self.state[0];

        // Calculate battery parameters based on current SoC
        let (r1, c1, r2, c2) = self.calculate_rc_parameters(soc);

        // Calculate time constants
        let tau1 = r1 * c1;
        let tau2 = r2 * c2;

        // State transition matrix A
        let a11 = 1.0;
        let a22 = libm::expf(-self.sampling_time / tau1);
        let a33 = libm::expf(-self.sampling_time / tau2);
        let a = [[a11, 0.0, 0.0], [0.0, a22, 0.0], [0.0, 0.0, a33]];

        // Input coefficient matrix B
        let b1 = -self.sampling_time * self.coulombic_efficiency / self.nominal_capacity;
        let b2 = r1 * (1.0 - libm::expf(-self.sampling_time / tau1));
        let b3 = r2 * (1.0 - libm::expf(-self.sampling_time / tau2));
        let b = [b1, b2, b3];

        // Predict state: x_k+1|k = A * x_k|k + B * u_k
        for i in 0..3 {
            self.predicted_state[i] = 0.0;
            for j in 0..3 {
                self.predicted_state[i] += a[i][j] * self.state[j];
            }
            self.predicted_state[i] += b[i] * current;
        }

        // Predict error covariance: P_k+1|k = A * P_k|k * A^T + Q
        let mut temp = [[0.0; 3]; 3];

        // temp = A * P
        for i in 0..3 {
            for j in 0..3 {
                temp[i][j] = 0.0;
                for k in 0..3 {
                    temp[i][j] += a[i][k] * self.error_covariance[k][j];
                }
            }
        }

        // P_predicted = temp * A^T + Q
        for i in 0..3 {
            for j in 0..3 {
                self.predicted_covariance[i][j] = 0.0;
                for k in 0..3 {
                    self.predicted_covariance[i][j] += temp[i][k] * a[j][k]; // A^T
                }
                // Add process noise
                if i == j {
                    self.predicted_covariance[i][j] += self.process_noise;
                }
            }
        }
    }

    /// Update step of the Extended Kalman Filter with measurement.
    fn update_with_measurement(&mut self, current: f32, terminal_voltage: f32) {
        let predicted_soc = self.predicted_state[0];
        let predicted_v_rc1 = self.predicted_state[1];
        let predicted_v_rc2 = self.predicted_state[2];

        // Calculate predicted measurement
        let r_int = self.calculate_internal_resistance(predicted_soc);
        let uocv = self.calculate_open_circuit_voltage(predicted_soc);
        let predicted_voltage = uocv - predicted_v_rc1 - predicted_v_rc2 - current * r_int;

        // Calculate measurement Jacobian (C matrix)
        let duocv_dsoc = self.calculate_uocv_derivative(predicted_soc);
        let c = [duocv_dsoc, -1.0, -1.0];

        // Calculate innovation covariance: S = C * P * C^T + R
        let mut cp = [0.0; 3];
        for i in 0..3 {
            cp[i] = 0.0;
            for j in 0..3 {
                cp[i] += c[j] * self.predicted_covariance[j][i];
            }
        }

        let mut s = 0.0;
        for i in 0..3 {
            s += cp[i] * c[i];
        }
        s += self.measurement_noise;

        // Calculate Kalman gain: K = P * C^T / S
        let mut kalman_gain = [0.0; 3];
        for i in 0..3 {
            kalman_gain[i] = cp[i] / s;
        }

        // Update state: x_k+1|k+1 = x_k+1|k + K * (y - h(x_k+1|k))
        let innovation = terminal_voltage - predicted_voltage;
        for i in 0..3 {
            self.state[i] = self.predicted_state[i] + kalman_gain[i] * innovation;
        }

        // Update error covariance: P_k+1|k+1 = (I - K * C) * P_k+1|k
        // First compute (I - K * C)
        let mut ikc = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let delta_ij = if i == j { 1.0 } else { 0.0 };
                ikc[i][j] = delta_ij - kalman_gain[i] * c[j];
            }
        }

        // Then multiply by P_predicted
        for i in 0..3 {
            for j in 0..3 {
                self.error_covariance[i][j] = 0.0;
                for k in 0..3 {
                    self.error_covariance[i][j] += ikc[i][k] * self.predicted_covariance[k][j];
                }
            }
        }
    }

    /// Calculates internal resistance as a function of SoC.
    pub fn calculate_internal_resistance(&self, soc: f32) -> f32 {
        let s = soc;
        let s2 = s * s;
        let s3 = s2 * s;
        let s4 = s3 * s;
        let s5 = s4 * s;
        let s6 = s5 * s;
        let s7 = s6 * s;
        0.00573 - 0.03427 * s + 0.1455 * s2 - 0.32647 * s3 + 0.41465 * s4 - 0.28992 * s5
            + 0.09353 * s6
            - 0.00634 * s7
    }

    /// Calculates RC circuit parameters as a function of SoC.
    pub fn calculate_rc_parameters(&self, soc: f32) -> (f32, f32, f32, f32) {
        let s = soc;
        let s2 = s * s;
        let s3 = s2 * s;
        let s4 = s3 * s;
        let s5 = s4 * s;
        let s6 = s5 * s;
        let s7 = s6 * s;

        // R1 and R2 (same polynomial)
        let r = 0.01513 - 0.18008 * s + 1.05147 * s2 - 3.27616 * s3 + 5.79793 * s4 - 5.81819 * s5
            + 3.08032 * s6
            - 0.66827 * s7;

        // C1 and C2 (same polynomial)
        let c = 47718.90713 - 1.00583e6 * s + 9.2653e6 * s2 - 3.91088e7 * s3 + 8.85892e7 * s4
            - 1.11014e8 * s5
            + 7.22811e7 * s6
            - 1.90336e7 * s7;

        (r, c, r, c)
    }

    /// Calculates open circuit voltage as a function of SoC.
    pub fn calculate_open_circuit_voltage(&self, soc: f32) -> f32 {
        let s = soc;
        let s2 = s * s;
        let s3 = s2 * s;
        let s4 = s3 * s;
        let s5 = s4 * s;
        let s6 = s5 * s;
        let s7 = s6 * s;
        -23.60229 * s7 + 141.34077 * s6 - 314.92282 * s5 + 345.34531 * s4 - 200.15462 * s3
            + 60.21383 * s2
            - 7.88447 * s
            + 3.2377
    }

    /// Calculates the derivative of open circuit voltage with respect to SoC.
    pub fn calculate_uocv_derivative(&self, soc: f32) -> f32 {
        let s = soc;
        let s2 = s * s;
        let s3 = s2 * s;
        let s4 = s3 * s;
        let s5 = s4 * s;
        let s6 = s5 * s;
        -23.60229 * 7.0 * s6 + 141.34077 * 6.0 * s5 - 314.92282 * 5.0 * s4 + 345.34531 * 4.0 * s3
            - 200.15462 * 3.0 * s2
            + 60.21383 * 2.0 * s
            - 7.88447
    }

    /// Resets the estimator to initial conditions.
    ///
    /// # Parameters
    ///
    /// * `initial_soc` - New initial State of Charge (0.0 to 1.0)
    pub fn reset(&mut self, initial_soc: f32) {
        self.state = [initial_soc.clamp(0.0, 1.0), 0.0, 0.0];
        self.error_covariance = [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]];
    }

    /// Gets the current error covariance for the SoC estimate.
    ///
    /// # Returns
    ///
    /// SoC estimation uncertainty (standard deviation)
    pub fn get_soc_uncertainty(&self) -> f32 {
        libm::sqrtf(self.error_covariance[0][0])
    }
}
