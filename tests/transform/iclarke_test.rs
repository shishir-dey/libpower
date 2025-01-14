#[cfg(test)]
mod tests {
    use libpower::transform::iclarke::*;

    #[test]
    fn test_iclarke_transform_balanced_inputs() {
        let mut iclarke = IClarke::new();
        iclarke.set_input(1.0, 0.0, 0.0); // Balanced alpha, beta, and zero
        iclarke.calculate();
        let (a, b, c) = iclarke.get_components();
        assert!((a - 1.0).abs() < 1e-6, "Phase A should be 1.0");
        assert!((b + 0.5).abs() < 1e-6, "Phase B should be -0.5");
        assert!((c + 0.5).abs() < 1e-6, "Phase C should be -0.5");
    }

    #[test]
    fn test_iclarke_transform_with_zero_offset() {
        let mut iclarke = IClarke::new();
        iclarke.set_input(1.0, 0.0, 0.5); // Includes zero-sequence component
        iclarke.calculate();
        let (a, b, c) = iclarke.get_components();
        assert!((a - 1.5).abs() < 1e-6, "Phase A should be 1.5");
        assert!((b + 0.0).abs() < 1e-6, "Phase B should be 0.0");
        assert!((c + 0.0).abs() < 1e-6, "Phase C should be 0.0");
    }

    #[test]
    fn test_iclarke_transform_unbalanced_inputs() {
        let mut iclarke = IClarke::new();
        iclarke.set_input(2.0, -1.0, 0.0); // Unbalanced alpha and beta
        iclarke.calculate();
        let (a, b, c) = iclarke.get_components();
        assert!((a - 2.0).abs() < 1e-6, "Phase A should be 2.0");
        assert!(
            (b - 0.366_025_4).abs() < 1e-6,
            "Phase B should be approximately 0.366"
        );
        assert!(
            (c + 1.633_975).abs() < 1e-6,
            "Phase C should be approximately -1.634"
        );
    }

    #[test]
    fn test_iclarke_transform_zero_inputs() {
        let mut iclarke = IClarke::new();
        iclarke.set_input(0.0, 0.0, 0.0); // All inputs are zero
        iclarke.calculate();
        let (a, b, c) = iclarke.get_components();
        assert!(a.abs() < 1e-6, "Phase A should be 0.0");
        assert!(b.abs() < 1e-6, "Phase B should be 0.0");
        assert!(c.abs() < 1e-6, "Phase C should be 0.0");
    }

    #[test]
    fn test_iclarke_transform_positive_inputs() {
        let mut iclarke = IClarke::new();
        iclarke.set_input(1.0, 1.0, 1.0); // All positive components
        iclarke.calculate();
        let (a, b, c) = iclarke.get_components();
        assert!((a - 2.0).abs() < 1e-6, "Phase A should be 2.0");
        assert!(
            (b - 1.366_025_4).abs() < 1e-6,
            "Phase B should be approximately 1.366"
        );
        assert!(
            (c - 0.633_974_6).abs() < 1e-6,
            "Phase C should be approximately 0.634"
        );
    }

    #[test]
    fn test_iclarke_transform_negative_inputs() {
        let mut iclarke = IClarke::new();
        iclarke.set_input(-1.0, -1.0, -1.0); // All negative components
        iclarke.calculate();
        let (a, b, c) = iclarke.get_components();
        assert!((a + 2.0).abs() < 1e-6, "Phase A should be -2.0");
        assert!(
            (b + 1.366_025_4).abs() < 1e-6,
            "Phase B should be approximately -1.366"
        );
        assert!(
            (c + 0.633_974_6).abs() < 1e-6,
            "Phase C should be approximately -0.634"
        );
    }

    #[test]
    fn test_iclarke_transform_large_inputs() {
        let mut iclarke = IClarke::new();
        iclarke.set_input(100.0, 50.0, 25.0); // Large inputs
        iclarke.calculate();
        let (a, b, c) = iclarke.get_components();
        assert!((a - 125.0).abs() < 1e-6, "Phase A should be 125.0");
        assert!(
            (b - 78.301_27).abs() < 1e-6,
            "Phase B should be approximately 78.301"
        );
        assert!(
            (c - 46.698_73).abs() < 1e-6,
            "Phase C should be approximately 46.699"
        );
    }

    #[test]
    fn test_iclarke_transform_with_phase_shifted_inputs() {
        let mut iclarke = IClarke::new();
        iclarke.set_input(0.0, 1.0, 0.0); // 90° shifted alpha and beta
        iclarke.calculate();
        let (a, b, c) = iclarke.get_components();
        assert!(a.abs() < 1e-6, "Phase A should be 0.0");
        assert!(
            (b - 0.866_025_4).abs() < 1e-6,
            "Phase B should be approximately 0.866"
        );
        assert!(
            (c + 0.866_025_4).abs() < 1e-6,
            "Phase C should be approximately -0.866"
        );
    }
}
