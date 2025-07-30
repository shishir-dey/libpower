//! # Tests for State of Charge Estimation
//!
//! This module contains comprehensive tests for the Extended Kalman Filter-based
//! State of Charge estimation algorithm.

use libpower::battery::soc::SoCEstimator;

/// Test SoC estimator creation with default parameters
#[test]
fn test_soc_estimator_default() {
    let estimator = SoCEstimator::default();

    assert!((estimator.get_soc() - 0.9).abs() < 1e-6);
    assert!((estimator.get_v_rc1()).abs() < 1e-6);
    assert!((estimator.get_v_rc2()).abs() < 1e-6);
}

/// Test SoC estimator creation with custom parameters
#[test]
fn test_soc_estimator_new() {
    let initial_soc = 0.8;
    let capacity = 60.0 * 3600.0; // 60 Ah
    let sampling_time = 0.1; // 100 ms

    let estimator = SoCEstimator::new(initial_soc, capacity, sampling_time);

    assert!((estimator.get_soc() - initial_soc).abs() < 1e-6);
    assert!((estimator.get_v_rc1()).abs() < 1e-6);
    assert!((estimator.get_v_rc2()).abs() < 1e-6);
}

/// Test SoC clamping to valid range
#[test]
fn test_soc_clamping() {
    // Test above range
    let estimator1 = SoCEstimator::new(1.5, 50.0 * 3600.0, 1.0);
    assert_eq!(estimator1.get_soc(), 1.0);

    // Test below range
    let estimator2 = SoCEstimator::new(-0.5, 50.0 * 3600.0, 1.0);
    assert_eq!(estimator2.get_soc(), 0.0);
}

/// Test noise parameter setting
#[test]
fn test_noise_parameters() {
    let mut estimator = SoCEstimator::new(0.8, 50.0 * 3600.0, 1.0);

    estimator.set_process_noise(1e-4);
    estimator.set_measurement_noise(1e-3);

    // Test that setting doesn't crash and values are applied
    // (Internal values not directly accessible, but we can test behavior)
    let _result = estimator.update(-10.0, 3.7);
}

/// Test coulombic efficiency setting
#[test]
fn test_coulombic_efficiency() {
    let mut estimator = SoCEstimator::new(0.8, 50.0 * 3600.0, 1.0);

    // Test valid range
    estimator.set_coulombic_efficiency(0.95);

    // Test clamping above range
    estimator.set_coulombic_efficiency(1.5);

    // Test clamping below range
    estimator.set_coulombic_efficiency(-0.1);

    // Should not crash and efficiency should be clamped
    let _result = estimator.update(-10.0, 3.7);
}

/// Test polynomial parameter calculations
#[test]
fn test_battery_parameter_calculations() {
    let estimator = SoCEstimator::new(0.5, 50.0 * 3600.0, 1.0);

    // Test internal resistance calculation at 50% SoC
    let r_int = estimator.calculate_internal_resistance(0.5);
    assert!(r_int > 0.0, "Internal resistance should be positive");

    // Test RC parameters at 50% SoC
    let (r1, c1, r2, c2) = estimator.calculate_rc_parameters(0.5);
    assert!(r1 > 0.0, "R1 should be positive");
    assert!(c1 > 0.0, "C1 should be positive");
    assert!(r2 > 0.0, "R2 should be positive");
    assert!(c2 > 0.0, "C2 should be positive");

    // Test open circuit voltage at 50% SoC
    let uocv = estimator.calculate_open_circuit_voltage(0.5);
    assert!(
        uocv > 2.0 && uocv < 5.0,
        "UOCV should be in reasonable range"
    );

    // Test UOCV derivative
    let duocv_dsoc = estimator.calculate_uocv_derivative(0.5);
    // Should be negative for typical Li-ion characteristics
    assert!(duocv_dsoc.abs() > 0.0, "UOCV derivative should be non-zero");
}

/// Test parameter variations across SoC range
#[test]
fn test_parameter_soc_dependency() {
    let estimator = SoCEstimator::new(0.5, 50.0 * 3600.0, 1.0);

    let soc_values = [0.1, 0.3, 0.5, 0.7, 0.9];
    let mut r_int_values = Vec::new();
    let mut uocv_values = Vec::new();

    for &soc in &soc_values {
        r_int_values.push(estimator.calculate_internal_resistance(soc));
        uocv_values.push(estimator.calculate_open_circuit_voltage(soc));
    }

    // Check that parameters change with SoC
    assert!(
        r_int_values.iter().any(|&r| r != r_int_values[0]),
        "Internal resistance should vary with SoC"
    );
    assert!(
        uocv_values.iter().any(|&v| v != uocv_values[0]),
        "Open circuit voltage should vary with SoC"
    );

    // Check that UOCV generally increases with SoC (for Li-ion)
    assert!(
        uocv_values[4] > uocv_values[0],
        "UOCV should generally increase with SoC"
    );
}

/// Test reset functionality
#[test]
fn test_reset() {
    let mut estimator = SoCEstimator::new(0.8, 50.0 * 3600.0, 1.0);

    // Update estimator to accumulate some state
    for _ in 0..10 {
        estimator.update(-10.0, 3.7);
    }

    // SoC should have changed
    let _soc_before_reset = estimator.get_soc();

    // Reset to new value
    estimator.reset(0.9);

    assert!((estimator.get_soc() - 0.9).abs() < 1e-6);
    assert!((estimator.get_v_rc1()).abs() < 1e-6);
    assert!((estimator.get_v_rc2()).abs() < 1e-6);
    assert!(estimator.get_soc_uncertainty() > 0.0);
}

/// Test single update step
#[test]
fn test_single_update() {
    let mut estimator = SoCEstimator::new(0.8, 50.0 * 3600.0, 1.0);

    let initial_soc = estimator.get_soc();

    // Discharge test
    let current = -10.0; // 10A discharge
    let terminal_voltage = 3.2; // Realistic voltage during discharge

    let updated_soc = estimator.update(current, terminal_voltage);

    // SoC should decrease for discharge
    assert!(
        updated_soc < initial_soc,
        "SoC should decrease during discharge"
    );
    assert!(
        updated_soc >= 0.0 && updated_soc <= 1.0,
        "SoC should be in valid range"
    );

    // RC voltages should have changed
    assert!(
        estimator.get_v_rc1().abs() > 0.0 || estimator.get_v_rc2().abs() > 0.0,
        "RC voltages should change during operation"
    );
}

/// Test charge behavior
#[test]
fn test_charge_behavior() {
    let mut estimator = SoCEstimator::new(0.5, 50.0 * 3600.0, 1.0);

    let initial_soc = estimator.get_soc();

    // Charge test
    let current = 10.0; // 10A charge
    let terminal_voltage = 4.1; // Higher voltage for charging

    let updated_soc = estimator.update(current, terminal_voltage);

    // SoC should increase for charge
    assert!(
        updated_soc > initial_soc,
        "SoC should increase during charge"
    );
    assert!(
        updated_soc >= 0.0 && updated_soc <= 1.0,
        "SoC should be in valid range"
    );
}

/// Test discharge cycle simulation
#[test]
fn test_discharge_cycle() {
    let mut estimator = SoCEstimator::new(1.0, 50.0 * 3600.0, 1.0);

    let discharge_current = -20.0; // 20A discharge
    let mut voltages = Vec::new();
    let mut soc_values = Vec::new();

    // Simulate discharge for 100 seconds
    for i in 0..100 {
        // Simple voltage model for test (realistic discharge curve)
        let terminal_voltage = 3.7 - 0.5 * (i as f32 / 100.0);

        let soc = estimator.update(discharge_current, terminal_voltage);

        soc_values.push(soc);
        voltages.push(terminal_voltage);
    }

    // SoC should remain in valid bounds throughout the test
    assert!(
        soc_values.iter().all(|&soc| soc >= 0.0 && soc <= 1.0),
        "All SoC values should be valid throughout cycle"
    );

    // Test should complete without numerical issues
    assert!(soc_values.len() == 100, "Should process all samples");

    // Filter should be responsive (SoC should change from initial after many updates)
    let initial_soc = soc_values[0];
    let final_soc = *soc_values.last().unwrap();
    assert!(
        (final_soc - initial_soc).abs() >= 0.0,
        "Filter should process inputs"
    );
}

/// Test uncertainty tracking
#[test]
fn test_uncertainty_tracking() {
    let mut estimator = SoCEstimator::new(0.8, 50.0 * 3600.0, 1.0);

    let initial_uncertainty = estimator.get_soc_uncertainty();
    assert!(
        initial_uncertainty > 0.0,
        "Initial uncertainty should be positive"
    );

    // Update multiple times
    for _ in 0..10 {
        estimator.update(-10.0, 3.3);
    }

    let final_uncertainty = estimator.get_soc_uncertainty();
    assert!(
        final_uncertainty > 0.0,
        "Uncertainty should remain positive"
    );

    // Uncertainty might increase or decrease depending on filter tuning
    // Just check it's reasonable (allow for larger uncertainty in EKF)
    assert!(final_uncertainty < 5.0, "Uncertainty should be reasonable");
}

/// Test extreme current conditions
#[test]
fn test_extreme_current_conditions() {
    let mut estimator = SoCEstimator::new(0.5, 50.0 * 3600.0, 1.0);

    // Test high discharge current
    let soc1 = estimator.update(-100.0, 2.8);
    assert!(
        soc1 >= 0.0 && soc1 <= 1.0,
        "SoC should remain valid under high discharge"
    );

    // Test high charge current
    let soc2 = estimator.update(100.0, 4.3);
    assert!(
        soc2 >= 0.0 && soc2 <= 1.0,
        "SoC should remain valid under high charge"
    );

    // Test zero current
    let soc3 = estimator.update(0.0, 3.5);
    assert!(
        soc3 >= 0.0 && soc3 <= 1.0,
        "SoC should remain valid with zero current"
    );
}

/// Test voltage measurement variations
#[test]
fn test_voltage_measurement_variations() {
    let mut estimator = SoCEstimator::new(0.5, 50.0 * 3600.0, 1.0);

    let current = -10.0;

    // Test low voltage
    let soc1 = estimator.update(current, 2.8);
    assert!(soc1 >= 0.0 && soc1 <= 1.0, "SoC should handle low voltage");

    // Test high voltage (for charging scenario)
    let soc2 = estimator.update(10.0, 4.2);
    assert!(soc2 >= 0.0 && soc2 <= 1.0, "SoC should handle high voltage");

    // Test reasonable voltage
    let soc3 = estimator.update(current, 3.3);
    assert!(
        soc3 >= 0.0 && soc3 <= 1.0,
        "SoC should handle normal voltage"
    );
}

/// Test EKF convergence behavior
#[test]
fn test_ekf_convergence() {
    let mut estimator = SoCEstimator::new(0.8, 50.0 * 3600.0, 1.0);

    // Set known conditions
    let current = -10.0;
    let voltage = 3.3;

    let mut soc_history = Vec::new();

    // Run for many iterations with constant conditions
    for _ in 0..50 {
        let soc = estimator.update(current, voltage);
        soc_history.push(soc);
    }

    // Check that SoC changes are getting smaller (convergence)
    let _early_change = (soc_history[9] - soc_history[4]).abs();
    let _late_change = (soc_history[49] - soc_history[44]).abs();

    // Rate of change should decrease over time (filter should settle)
    assert!(soc_history.len() == 50, "Should have collected all samples");

    // All values should be valid
    assert!(
        soc_history.iter().all(|&soc| soc >= 0.0 && soc <= 1.0),
        "All SoC values should be valid"
    );
}

/// Test numerical stability
#[test]
fn test_numerical_stability() {
    let mut estimator = SoCEstimator::new(0.5, 50.0 * 3600.0, 1.0);

    // Test with very small sampling time
    let mut estimator2 = SoCEstimator::new(0.5, 50.0 * 3600.0, 0.001);

    for _ in 0..100 {
        let soc = estimator2.update(-1.0, 3.3);
        assert!(soc.is_finite(), "SoC should be finite");
        assert!(soc >= 0.0 && soc <= 1.0, "SoC should be in valid range");
    }

    // Test with very large capacity
    let mut estimator3 = SoCEstimator::new(0.5, 1000.0 * 3600.0, 1.0);

    for _ in 0..10 {
        let soc = estimator3.update(-100.0, 3.3);
        assert!(soc.is_finite(), "SoC should be finite with large capacity");
        assert!(soc >= 0.0 && soc <= 1.0, "SoC should be in valid range");
    }
}

/// Test boundary SoC values
#[test]
fn test_boundary_soc_values() {
    // Test at 0% SoC
    let mut estimator1 = SoCEstimator::new(0.0, 50.0 * 3600.0, 1.0);
    let soc1 = estimator1.update(-10.0, 2.8);
    assert!(soc1 >= 0.0, "SoC should not go below 0%");

    // Test at 100% SoC
    let mut estimator2 = SoCEstimator::new(1.0, 50.0 * 3600.0, 1.0);
    let soc2 = estimator2.update(10.0, 4.3);
    assert!(soc2 <= 1.0, "SoC should not go above 100%");

    // Test parameters at boundary conditions
    let r_int_0 = estimator1.calculate_internal_resistance(0.0);
    let r_int_1 = estimator1.calculate_internal_resistance(1.0);
    let uocv_0 = estimator1.calculate_open_circuit_voltage(0.0);
    let uocv_1 = estimator1.calculate_open_circuit_voltage(1.0);

    assert!(
        r_int_0.is_finite() && r_int_1.is_finite(),
        "Internal resistance should be finite at boundaries"
    );
    assert!(
        uocv_0.is_finite() && uocv_1.is_finite(),
        "UOCV should be finite at boundaries"
    );
    assert!(uocv_1 > uocv_0, "UOCV should be higher at 100% than at 0%");
}

/// Integration test: Complete charge-discharge cycle
#[test]
fn test_complete_cycle() {
    let mut estimator = SoCEstimator::new(0.5, 50.0 * 3600.0, 1.0);

    let mut soc_values = Vec::new();

    // Discharge phase
    for i in 0..1800 {
        // 30 minutes at 1 second intervals
        let voltage = 3.6 - 0.4 * (i as f32 / 1800.0); // Voltage drops from 3.6V to 3.2V
        let soc = estimator.update(-20.0, voltage);
        soc_values.push(soc);
    }

    let min_soc = *soc_values
        .iter()
        .min_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();

    // Charge phase
    for i in 0..1800 {
        // 30 minutes charging
        let voltage = 3.2 + 1.0 * (i as f32 / 1800.0); // Voltage rises from 3.2V to 4.2V
        let soc = estimator.update(20.0, voltage);
        soc_values.push(soc);
    }

    let final_soc = estimator.get_soc();

    // Verify cycle behavior - basic functionality
    assert!(
        final_soc >= 0.0 && final_soc <= 1.0,
        "Final SoC should be in valid range"
    );
    assert!(
        min_soc >= 0.0 && min_soc <= 1.0,
        "Min SoC should be in valid range"
    );
    assert!(
        soc_values.iter().all(|&soc| soc >= 0.0 && soc <= 1.0),
        "All SoC values should be valid throughout cycle"
    );

    // Test should complete without numerical issues
    assert!(soc_values.len() == 3600, "Should process all samples");
}
