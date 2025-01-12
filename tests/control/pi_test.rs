#[cfg(test)]
mod tests {
    use libpower::control::cntl_pi::*;

    #[test]
    fn test_controller_initialization() {
        let controller = ControllerPI::new();
        assert_eq!(controller.get_output(), 0.0);
        assert_eq!(controller.get_proportional_term(), 0.0);
        assert_eq!(controller.get_integral_term(), 0.0);
    }

    #[test]
    fn test_with_gains() {
        let controller = ControllerPI::with_gains(0.5, 0.3);
        assert_eq!(controller.get_kp(), 0.5);
        assert_eq!(controller.get_ki(), 0.3);
    }

    #[test]
    fn test_proportional_response() {
        let mut controller = ControllerPI::with_gains(1.0, 0.0); // P-only controller
        let output = controller.calculate(1.0, 0.0); // Error = 1.0
        assert_eq!(output, 1.0); // Output should equal error * Kp
    }

    #[test]
    fn test_saturation_limits() {
        let mut controller = ControllerPI::with_gains(1.0, 0.1);
        controller.set_limits(-0.5, 0.5);

        // Test upper limit
        let output = controller.calculate(1.0, 0.0);
        assert!(output <= 0.5);

        // Test lower limit
        let output = controller.calculate(-1.0, 0.0);
        assert!(output >= -0.5);
    }

    #[test]
    fn test_reset() {
        let mut controller = ControllerPI::with_gains(1.0, 0.1);
        controller.calculate(1.0, 0.0);
        controller.reset();

        assert_eq!(controller.get_output(), 0.0);
        assert_eq!(controller.get_proportional_term(), 0.0);
        assert_eq!(controller.get_integral_term(), 0.0);
    }

    #[test]
    fn test_integral_action() {
        let mut controller = ControllerPI::with_gains(0.0, 1.0); // I-only controller
        let mut output = 0.0;

        // Constant error should lead to ramping output
        for _ in 0..3 {
            output = controller.calculate(1.0, 0.0); // Constant error of 1.0
        }

        assert!(output > 2.0); // Output should accumulate over time
    }
}
