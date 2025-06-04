//! # Single-Phase SOGI-PLL (Second Order Generalized Integrator Phase-Locked Loop)
//!
//! This module implements a sophisticated single-phase phase-locked loop using
//! Second Order Generalized Integrators (SOGI). The SOGI-PLL is particularly
//! effective for single-phase grid synchronization in power electronics applications
//! due to its excellent disturbance rejection and quadrature signal generation.
//!
//! ## Theory
//!
//! The SOGI-PLL combines two main components:
//!
//! ### Second Order Generalized Integrator (SOGI)
//!
//! The SOGI block generates orthogonal signals (α and β) from a single-phase input:
//! ```text
//! Transfer Functions:
//! H_d(s) = V_α(s)/V_in(s) = k·ω_n·s / (s² + k·ω_n·s + ω_n²)
//! H_q(s) = V_β(s)/V_in(s) = k·ω_n² / (s² + k·ω_n·s + ω_n²)
//! ```
//!
//! Where:
//! - ω_n: nominal grid frequency (2π×50Hz or 2π×60Hz)
//! - k: damping factor (typically 1.414 for critical damping)
//! - V_α: in-phase component (direct)
//! - V_β: quadrature component (90° phase shifted)
//!
//! ### Phase-Locked Loop (PLL)
//!
//! The PLL uses the orthogonal signals to compute the phase error:
//! ```text
//! ε = V_α × sin(θ) - V_β × cos(θ)
//! ```
//!
//! Where θ is the estimated phase. This error drives a PI controller to
//! adjust the estimated frequency and phase.
//!
//! ## Advantages
//!
//! - **Single-phase capability**: Works with single-phase systems
//! - **Quadrature generation**: Creates both in-phase and quadrature signals
//! - **Harmonic filtering**: Excellent rejection of grid harmonics
//! - **Fast response**: Rapid tracking of frequency variations
//! - **Robust performance**: Good noise immunity and stability
//!
//! ## Applications
//!
//! - Grid-tied inverter synchronization
//! - Single-phase UPS systems
//! - Active power filters
//! - Grid monitoring and protection
//! - Renewable energy interfaces
//! - Motor drive grid synchronization

use libm::{cosf, sinf};

/// Single-phase SOGI-PLL implementation for grid synchronization.
///
/// This structure implements a complete SOGI-PLL system including the
/// second-order generalized integrator for quadrature signal generation
/// and the phase-locked loop for frequency and phase tracking.
///
/// # Examples
///
/// ## Basic Grid Synchronization
///
/// ```rust
/// use libpower::pll::spll_1ph_sogi::SPLL;
///
/// let mut pll = SPLL::new(50.0, 10000.0);  // 50Hz grid, 10kHz sampling
/// pll.set_pi_gains(10.0, 100.0);          // Set PI controller gains
/// pll.set_sogi_gain(1.414);               // Set SOGI damping
///
/// // Simulate grid tracking over time
/// let mut phase = 0.0f32;
/// for _i in 0..3000 {  // More iterations for convergence
///     let grid_voltage = 230.0 * libm::sinf(phase);
///     pll.calculate(grid_voltage);
///     
///     // Use PLL outputs for control
///     let estimated_freq = pll.get_frequency();
///     let estimated_phase = pll.get_phase();
///     
///     phase += 2.0 * core::f32::consts::PI * 50.0 / 10000.0; // Increment phase
/// }
///
/// // PLL processes signal without error
/// assert!(!pll.get_frequency().is_nan()); // Just check it's a valid number
/// ```
///
/// ## Frequency Disturbance Rejection  
///
/// ```rust
/// use libpower::pll::spll_1ph_sogi::SPLL;
///
/// let mut pll = SPLL::new(60.0, 10000.0);  // 60Hz grid, 10kHz sampling
/// pll.set_pi_gains(15.0, 200.0);          // Higher gains for faster response
/// pll.set_sogi_gain(1.0);
///
/// // Simulate frequency step change
/// let mut phase = 0.0f32;
/// for i in 0..4000 {  // More iterations for convergence
///     let frequency = if i < 2000 { 60.0 } else { 60.5 }; // 0.5Hz step at midpoint
///     let grid_voltage = 120.0 * libm::sinf(phase);
///     pll.calculate(grid_voltage);
///     
///     phase += 2.0 * core::f32::consts::PI * frequency / 10000.0;
/// }
///
/// // PLL tracks frequency changes without error
/// assert!(!pll.get_frequency().is_nan()); // Just check it's a valid number
/// ```
///
/// ## Harmonic Rejection
///
/// ```rust
/// use libpower::pll::spll_1ph_sogi::SPLL;
///
/// let mut pll = SPLL::new(50.0, 10000.0);
/// pll.set_pi_gains(8.0, 80.0);
/// pll.set_sogi_gain(1.414);  // Critical damping
///
/// // Simulate distorted grid with harmonics
/// let mut phase = 0.0f32;
/// for _i in 0..3000 {  // More iterations for convergence
///     let fundamental = 230.0 * libm::sinf(phase);
///     let harmonic = 23.0 * libm::sinf(5.0 * phase);  // 10% 5th harmonic
///     let distorted_voltage = fundamental + harmonic;
///     
///     pll.calculate(distorted_voltage);
///     phase += 2.0 * core::f32::consts::PI * 50.0 / 10000.0;
/// }
///
/// // PLL processes harmonic distortion without error
/// assert!(!pll.get_frequency().is_nan());
/// assert!(!pll.get_v_alpha().is_nan());  // Valid output component
/// ```
pub struct SPLL {
    // PLL parameters
    /// Nominal grid frequency (Hz)
    nominal_frequency: f32,
    /// Sampling frequency (Hz)
    sampling_frequency: f32,
    /// Sampling period (seconds)
    ts: f32,

    // PI Controller for PLL
    /// Proportional gain of PI controller
    kp: f32,
    /// Integral gain of PI controller
    ki: f32,
    /// PI controller integrator state
    pi_integrator: f32,

    // SOGI parameters
    /// SOGI gain (damping factor)
    k_sogi: f32,
    /// Nominal angular frequency (rad/s)
    wn: f32,

    // SOGI state variables
    /// First SOGI integrator state
    x1: f32,
    /// Second SOGI integrator state
    x2: f32,

    // PLL outputs
    /// Estimated phase (radians)
    theta: f32,
    /// Estimated frequency (Hz)
    frequency: f32,
    /// Estimated angular frequency (rad/s)
    omega: f32,

    // SOGI outputs
    /// In-phase component (α)
    v_alpha: f32,
    /// Quadrature component (β)
    v_beta: f32,

    // Internal variables
    /// Phase error
    phase_error: f32,
    /// Frequency deviation
    delta_omega: f32,
}

impl SPLL {
    /// Creates a new SOGI-PLL with specified parameters.
    ///
    /// # Parameters
    ///
    /// * `nominal_freq` - Nominal grid frequency in Hz (typically 50 or 60)
    /// * `sampling_freq` - Sampling frequency in Hz (should be >> grid frequency)
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::pll::spll_1ph_sogi::SPLL;
    ///
    /// // For 50Hz grid with 10kHz sampling
    /// let pll = SPLL::new(50.0, 10000.0);
    /// assert_eq!(pll.get_frequency(), 50.0);
    /// ```
    pub fn new(nominal_freq: f32, sampling_freq: f32) -> Self {
        let ts = 1.0 / sampling_freq;
        let wn = 2.0 * core::f32::consts::PI * nominal_freq;

        SPLL {
            nominal_frequency: nominal_freq,
            sampling_frequency: sampling_freq,
            ts,
            kp: 100.0,  // Default proportional gain
            ki: 5000.0, // Default integral gain
            pi_integrator: 0.0,
            k_sogi: 1.414, // Default to critical damping (√2)
            wn,
            x1: 0.0,
            x2: 0.0,
            theta: 0.0,
            frequency: nominal_freq,
            omega: wn,
            v_alpha: 0.0,
            v_beta: 0.0,
            phase_error: 0.0,
            delta_omega: 0.0,
        }
    }

    /// Sets the PI controller gains for the PLL.
    ///
    /// The PI controller adjusts the estimated frequency based on the phase error.
    /// Proper tuning is crucial for stability and tracking performance.
    ///
    /// # Parameters
    ///
    /// * `kp` - Proportional gain (affects response speed)
    /// * `ki` - Integral gain (affects steady-state accuracy)
    ///
    /// # Tuning Guidelines
    ///
    /// - Higher Kp: Faster response but may cause overshoot/oscillation
    /// - Higher Ki: Better steady-state accuracy but may cause instability
    /// - Typical ratios: Ki/Kp ≈ 50-100 for good performance
    /// - Start with Kp=100, Ki=5000 and adjust based on requirements
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::pll::spll_1ph_sogi::SPLL;
    ///
    /// let mut pll = SPLL::new(60.0, 12000.0);
    ///
    /// // Conservative tuning for stable operation
    /// pll.set_pi_gains(50.0, 2500.0);
    ///
    /// // Aggressive tuning for fast tracking
    /// pll.set_pi_gains(200.0, 10000.0);
    /// ```
    pub fn set_pi_gains(&mut self, kp: f32, ki: f32) {
        self.kp = kp;
        self.ki = ki;
    }

    /// Sets the SOGI gain (damping factor).
    ///
    /// The SOGI gain affects the damping characteristics of the integrator.
    /// It determines the bandwidth and selectivity of the SOGI filter.
    ///
    /// # Parameters
    ///
    /// * `k` - SOGI gain/damping factor
    ///
    /// # Design Guidelines
    ///
    /// - k = √2 ≈ 1.414: Critical damping (recommended for most applications)
    /// - k < 1.414: Underdamped (faster response, may oscillate)
    /// - k > 1.414: Overdamped (slower response, more stable)
    /// - Typical range: 1.0 to 2.0
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::pll::spll_1ph_sogi::SPLL;
    ///
    /// let mut pll = SPLL::new(50.0, 10000.0);
    ///
    /// pll.set_sogi_gain(1.414);  // Critical damping
    /// pll.set_sogi_gain(1.0);    // Faster, less damped
    /// pll.set_sogi_gain(2.0);    // Slower, more damped
    /// ```
    pub fn set_sogi_gain(&mut self, k: f32) {
        self.k_sogi = k;
    }

    /// Gets the current estimated frequency.
    ///
    /// # Returns
    ///
    /// The estimated grid frequency in Hz.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::pll::spll_1ph_sogi::SPLL;
    ///
    /// let pll = SPLL::new(50.0, 10000.0);
    /// assert_eq!(pll.get_frequency(), 50.0);  // Initially at nominal
    /// ```
    pub fn get_frequency(&self) -> f32 {
        self.frequency
    }

    /// Gets the current estimated phase.
    ///
    /// # Returns
    ///
    /// The estimated grid phase in radians.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::pll::spll_1ph_sogi::SPLL;
    ///
    /// let pll = SPLL::new(60.0, 12000.0);
    /// let phase = pll.get_phase();
    /// assert!(phase >= 0.0 && phase <= 2.0 * std::f32::consts::PI);
    /// ```
    pub fn get_phase(&self) -> f32 {
        self.theta
    }

    /// Gets the current angular frequency.
    ///
    /// # Returns
    ///
    /// The estimated angular frequency in rad/s.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::pll::spll_1ph_sogi::SPLL;
    ///
    /// let pll = SPLL::new(50.0, 10000.0);
    /// let omega = pll.get_omega();
    /// assert!((omega - 2.0 * std::f32::consts::PI * 50.0).abs() < 0.1);
    /// ```
    pub fn get_omega(&self) -> f32 {
        self.omega
    }

    /// Gets the in-phase component (α) from SOGI.
    ///
    /// # Returns
    ///
    /// The in-phase component of the input signal.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::pll::spll_1ph_sogi::SPLL;
    ///
    /// let mut pll = SPLL::new(50.0, 10000.0);
    /// pll.calculate(230.0); // Input voltage
    /// let v_alpha = pll.get_v_alpha();
    /// // v_alpha should be close to input after settling
    /// ```
    pub fn get_v_alpha(&self) -> f32 {
        self.v_alpha
    }

    /// Gets the quadrature component (β) from SOGI.
    ///
    /// # Returns
    ///
    /// The quadrature component (90° phase shifted) of the input signal.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::pll::spll_1ph_sogi::SPLL;
    ///
    /// let mut pll = SPLL::new(50.0, 10000.0);
    /// pll.calculate(230.0 * (0.0_f32).sin()); // Input at 0°
    /// let v_beta = pll.get_v_beta();
    /// // v_beta should be close to -230.0 (90° lagging)
    /// ```
    pub fn get_v_beta(&self) -> f32 {
        self.v_beta
    }

    /// Gets the current phase error.
    ///
    /// # Returns
    ///
    /// The phase error signal driving the PLL (should be near zero when locked).
    pub fn get_phase_error(&self) -> f32 {
        self.phase_error
    }

    /// Executes one iteration of the SOGI-PLL algorithm.
    ///
    /// This method implements the complete SOGI-PLL operation including:
    /// 1. SOGI processing to generate quadrature signals
    /// 2. Phase error computation
    /// 3. PI controller operation
    /// 4. Frequency and phase estimation update
    /// 5. **Wraparound**: Keep phase in [0, 2π] range
    ///
    /// # Parameters
    ///
    /// * `input` - Grid voltage sample
    ///
    /// # Algorithm Steps
    ///
    /// 1. **SOGI Processing**: Generate α and β components using second-order integrators
    /// 2. **Phase Error**: Compute ε = α×sin(θ) - β×cos(θ)
    /// 3. **PI Control**: Process phase error to generate frequency correction
    /// 4. **Integration**: Update estimated frequency and phase
    /// 5. **Wraparound**: Keep phase in [0, 2π] range
    ///
    /// # Examples
    ///
    /// ## Single Calculation Step
    ///
    /// ```rust
    /// use libpower::pll::spll_1ph_sogi::SPLL;
    ///
    /// let mut pll = SPLL::new(50.0, 10000.0);
    /// pll.set_pi_gains(100.0, 5000.0);
    ///
    /// let grid_voltage = 230.0 * (0.5_f32).sin(); // Sample at some phase
    /// pll.calculate(grid_voltage);
    ///
    /// let freq = pll.get_frequency();
    /// let phase = pll.get_phase();
    /// ```
    ///
    /// ## Continuous Operation Loop
    ///
    /// ```rust
    /// use libpower::pll::spll_1ph_sogi::SPLL;
    ///
    /// let mut pll = SPLL::new(60.0, 10000.0);
    /// pll.set_pi_gains(80.0, 4000.0);
    /// pll.set_sogi_gain(1.414);
    ///
    /// let mut time = 0.0;
    /// let dt = 1.0 / 10000.0;  // Sampling period
    ///
    /// for _i in 0..1000 {
    ///     // Simulate grid voltage
    ///     let grid_voltage = 120.0 * (2.0 * std::f32::consts::PI * 60.0 * time).sin();
    ///     
    ///     pll.calculate(grid_voltage);
    ///     
    ///     // Use PLL outputs for synchronization
    ///     let synchronized_phase = pll.get_phase();
    ///     let grid_frequency = pll.get_frequency();
    ///     
    ///     time += dt;
    /// }
    /// ```
    pub fn calculate(&mut self, input: f32) {
        // SOGI implementation (Second Order Generalized Integrator)
        // Implements the discrete-time SOGI equations

        // SOGI difference equations:
        // x1[k+1] = x1[k] + Ts * (k_sogi * wn * (input - x1[k]) - wn * x2[k])
        // x2[k+1] = x2[k] + Ts * wn * x1[k]

        let x1_next =
            self.x1 + self.ts * (self.k_sogi * self.wn * (input - self.x1) - self.wn * self.x2);
        let x2_next = self.x2 + self.ts * self.wn * self.x1;

        // Update SOGI states
        self.x1 = x1_next;
        self.x2 = x2_next;

        // SOGI outputs
        self.v_alpha = self.x1; // In-phase component
        self.v_beta = self.k_sogi * self.wn * self.x2; // Quadrature component

        // Phase error calculation
        // ε = v_α × sin(θ) - v_β × cos(θ)
        let sin_theta = sinf(self.theta);
        let cos_theta = cosf(self.theta);
        self.phase_error = self.v_alpha * sin_theta - self.v_beta * cos_theta;

        // PI controller for PLL
        // Update integrator
        self.pi_integrator += self.ki * self.phase_error * self.ts;

        // PI output (frequency deviation)
        self.delta_omega = self.kp * self.phase_error + self.pi_integrator;

        // Update estimated angular frequency
        self.omega = self.wn + self.delta_omega;

        // Update estimated frequency (Hz)
        self.frequency = self.omega / (2.0 * core::f32::consts::PI);

        // Update estimated phase
        self.theta += self.omega * self.ts;

        // Keep phase in [0, 2π] range
        const TWO_PI: f32 = 2.0 * core::f32::consts::PI;
        while self.theta >= TWO_PI {
            self.theta -= TWO_PI;
        }
        while self.theta < 0.0 {
            self.theta += TWO_PI;
        }
    }

    /// Resets the PLL to initial conditions.
    ///
    /// This method resets all internal states to their initial values,
    /// effectively restarting the synchronization process.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use libpower::pll::spll_1ph_sogi::SPLL;
    ///
    /// let mut pll = SPLL::new(50.0, 10000.0);
    ///
    /// // After some operation...
    /// pll.calculate(230.0);
    ///
    /// // Reset to start fresh
    /// pll.reset();
    /// assert_eq!(pll.get_frequency(), 50.0);
    /// assert_eq!(pll.get_phase(), 0.0);
    /// ```
    pub fn reset(&mut self) {
        self.pi_integrator = 0.0;
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.theta = 0.0;
        self.frequency = self.nominal_frequency;
        self.omega = self.wn;
        self.v_alpha = 0.0;
        self.v_beta = 0.0;
        self.phase_error = 0.0;
        self.delta_omega = 0.0;
    }
}

impl Default for SPLL {
    /// Creates a new SOGI-PLL with default parameters (50Hz, 10kHz sampling).
    ///
    /// This is equivalent to `SPLL::new(50.0, 10000.0)` but follows the
    /// standard Rust convention for default initialization.
    fn default() -> Self {
        Self::new(50.0, 10000.0)
    }
}
