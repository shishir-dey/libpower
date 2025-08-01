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
//! // Create estimator with default battery parameters (Li-Ion NMC 18650 3.7V 2200mAh 2C)
//! let mut estimator = Battery::new(0.9, 50.0 * 3600.0, 1.0);
//! estimator.set_process_noise(1e-5);
//! estimator.set_measurement_noise(5e-4);
//!
//! // Or create with specific battery type
//! let mut nmc_18650_3_7v_2500mah_2c_estimator = Battery::with_battery_type(
//!     0.9,
//!     50.0 * 3600.0,
//!     1.0,
//!     "NMC_18650_3.7V_2500mAh_2C"
//! ).unwrap();
//!
//! // In your control loop
//! let current = -10.0; // 10A discharge
//! let terminal_voltage = 3.7; // Measured terminal voltage
//!
//! let estimated_soc = estimator.update(current, terminal_voltage);
//! println!("Estimated SoC: {:.1}%", estimated_soc * 100.0);
//!
//! // Get battery information
//! let battery_info = estimator.get_battery_info();
//! println!("Battery: {} {} {}",
//!          battery_info.manufacturer,
//!          battery_info.chemistry,
//!          battery_info.form_factor);
//! ```
//!
//! ## Adding New Battery Types
//!
//! To add support for a new battery type:
//! 1. Create a new TOML file in `src/battery/soc/params/` with your battery parameters
//! 2. Add a new match case in `BatteryParameters::load_battery_type()`
//! 3. The TOML file should contain sections for:
//!    - `[battery_info]` - Battery metadata
//!    - `[internal_resistance]` - Internal resistance polynomial coefficients
//!    - `[rc_resistance]` - RC circuit resistance coefficients  
//!    - `[rc_capacitance]` - RC circuit capacitance coefficients
//!    - `[open_circuit_voltage]` - Open circuit voltage coefficients

use libm;
use serde::{Deserialize, Serialize};

extern crate alloc;
use alloc::string::String;

/// Battery information and metadata
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct BatteryInfo {
    pub manufacturer: String,
    pub chemistry: String,
    pub form_factor: String,
    pub nominal_voltage: f32,  // V
    pub nominal_capacity: f32, // mAh
}

/// Polynomial coefficients for battery parameter calculations
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct PolynomialCoefficients {
    pub coefficients: [f32; 8], // a0 to a7
}

/// Complete battery parameter set loaded from TOML
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct BatteryParameters {
    pub battery_info: BatteryInfo,
    pub internal_resistance: PolynomialCoefficients,
    pub rc_resistance: PolynomialCoefficients,
    pub rc_capacitance: PolynomialCoefficients,
    pub open_circuit_voltage: PolynomialCoefficients,
}

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

    // Battery-specific parameters loaded from TOML
    battery_parameters: BatteryParameters,
}

impl BatteryParameters {
    /// Load battery parameters from a TOML string
    pub fn from_toml_str(toml_str: &str) -> Result<Self, &'static str> {
        toml::from_str(toml_str).map_err(|_| "Failed to parse TOML")
    }

    /// Load battery parameters from embedded TOML data
    pub fn load_default() -> Self {
        const DEFAULT_TOML: &str = include_str!("params/NMC_18650_3.7V_2500mAh_2C.toml");
        Self::from_toml_str(DEFAULT_TOML).expect("Default battery parameters should be valid")
    }

    /// Load a specific battery type by filename (without .toml extension)
    pub fn load_battery_type(filename: &str) -> Result<Self, &'static str> {
        match filename {
            "NMC_18650_3.7V_2500mAh_2C" => {
                const TOML_DATA: &str = include_str!("params/NMC_18650_3.7V_2500mAh_2C.toml");
                Self::from_toml_str(TOML_DATA)
            }
            _ => Err("Battery type not found"),
        }
    }
}

/// Helper function to evaluate polynomial: a0 + a1*x + a2*x^2 + ... + a7*x^7
fn evaluate_polynomial(coefficients: &[f32; 8], x: f32) -> f32 {
    let mut result = coefficients[0];
    let mut x_power = x;

    for i in 1..8 {
        result += coefficients[i] * x_power;
        x_power *= x;
    }

    result
}

/// Helper function to evaluate polynomial derivative: a1 + 2*a2*x + 3*a3*x^2 + ... + 7*a7*x^6
fn evaluate_polynomial_derivative(coefficients: &[f32; 8], x: f32) -> f32 {
    let mut result = coefficients[1];
    let mut x_power = 1.0;

    for i in 2..8 {
        result += (i as f32) * coefficients[i] * x_power;
        x_power *= x;
    }

    result
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
    /// - Uses default battery parameters (Li-Ion NMC 18650 3.7V 2200mAh 2C)
    fn default() -> Self {
        Self::new(0.9, 50.0 * 3600.0, 1.0) // 50 Ah in As
    }
}

impl Battery {
    /// Creates a new SoC estimator with specified parameters using default battery.
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
        Self::with_battery_type(
            initial_soc,
            nominal_capacity,
            sampling_time,
            "NMC_18650_3.7V_2500mAh_2C",
        )
        .expect("Default battery parameters should be valid")
    }

    /// Creates a new SoC estimator with specified battery type.
    ///
    /// # Parameters
    ///
    /// * `initial_soc` - Initial State of Charge (0.0 to 1.0)
    /// * `nominal_capacity` - Battery nominal capacity in Ampere-seconds (As)
    /// * `sampling_time` - Sampling period in seconds
    /// * `battery_type` - Battery type identifier (filename without .toml extension)
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::battery::soc::Battery;
    ///
    /// // Create estimator with specific battery type
    /// let estimator = Battery::with_battery_type(
    ///     0.8,
    ///     50.0 * 3600.0,
    ///     1.0,
    ///     "NMC_18650_3.7V_2200mAh_2C"
    /// ).unwrap();
    /// ```
    pub fn with_battery_type(
        initial_soc: f32,
        nominal_capacity: f32,
        sampling_time: f32,
        battery_type: &str,
    ) -> Result<Self, &'static str> {
        let battery_parameters = BatteryParameters::load_battery_type(battery_type)?;

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
            battery_parameters,
        };

        // Ensure SoC is within valid range
        estimator.state[0] = estimator.state[0].clamp(0.0, 1.0);
        Ok(estimator)
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

    /// Gets the battery information and metadata.
    ///
    /// # Returns
    ///
    /// Reference to the battery information
    pub fn get_battery_info(&self) -> &BatteryInfo {
        &self.battery_parameters.battery_info
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

    /// Calculates internal resistance as a function of SoC using loaded coefficients.
    pub fn calculate_internal_resistance(&self, soc: f32) -> f32 {
        evaluate_polynomial(
            &self.battery_parameters.internal_resistance.coefficients,
            soc,
        )
    }

    /// Calculates RC circuit parameters as a function of SoC using loaded coefficients.
    pub fn calculate_rc_parameters(&self, soc: f32) -> (f32, f32, f32, f32) {
        // Both R1 and R2 use the same polynomial
        let r = evaluate_polynomial(&self.battery_parameters.rc_resistance.coefficients, soc);

        // Both C1 and C2 use the same polynomial
        let c = evaluate_polynomial(&self.battery_parameters.rc_capacitance.coefficients, soc);

        (r, c, r, c)
    }

    /// Calculates open circuit voltage as a function of SoC using loaded coefficients.
    pub fn calculate_open_circuit_voltage(&self, soc: f32) -> f32 {
        evaluate_polynomial(
            &self.battery_parameters.open_circuit_voltage.coefficients,
            soc,
        )
    }

    /// Calculates the derivative of open circuit voltage with respect to SoC using loaded coefficients.
    pub fn calculate_uocv_derivative(&self, soc: f32) -> f32 {
        evaluate_polynomial_derivative(
            &self.battery_parameters.open_circuit_voltage.coefficients,
            soc,
        )
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
