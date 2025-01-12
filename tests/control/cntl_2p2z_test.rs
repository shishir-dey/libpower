// Tests module
#[cfg(test)]
mod tests {
    use libpower::control::cntl_2p2z::*;

    #[test]
    fn test_controller_initialization() {
        let coeffs = Coefficients::with_default_values();
        let controller = Controller2p2z::new(coeffs);
        assert_eq!(controller.get_output(), 0.0);
    }

    #[test]
    fn test_error_calculation() {
        let coeffs = Coefficients::with_default_values();
        let mut controller = Controller2p2z::new(coeffs);
        controller.calculate(1.0, 0.1);
        assert!((controller.get_error() - 0.9).abs() < 1e-6);
    }

    #[test]
    fn test_output_saturation() {
        let mut coeffs = Coefficients::with_default_values();
        coeffs.max = 1.0;
        coeffs.min = -1.0;
        let mut controller = Controller2p2z::new(coeffs);

        // Test maximum saturation
        let out = controller.calculate(10.0, 0.0);
        assert!(out <= 1.0);

        // Test minimum saturation
        let out = controller.calculate(-10.0, 0.0);
        assert!(out >= -1.0);
    }

    #[test]
    fn test_controller_reset() {
        let coeffs = Coefficients::with_default_values();
        let mut controller = Controller2p2z::new(coeffs);
        controller.calculate(1.0, 0.1);
        controller.reset();
        assert_eq!(controller.get_output(), 0.0);
        assert_eq!(controller.get_error(), 0.0);
    }

    #[test]
    fn test_default_coefficient_values() {
        let coeffs = Coefficients::with_default_values();
        assert_eq!(coeffs.coeff_b2, 0.3);
        assert_eq!(coeffs.coeff_b1, 0.2);
        assert_eq!(coeffs.coeff_b0, 0.1);
        assert_eq!(coeffs.coeff_a2, 0.2);
        assert_eq!(coeffs.coeff_a1, 0.1);
    }
}
