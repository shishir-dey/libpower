#[cfg(test)]
mod tests {
    use libpower::transform::clarke::*;

    #[test]
    fn test_clarke_transform_balanced_inputs() {
        let mut clarke = Clarke::new();
        clarke.set_input(1.0, -0.5, -0.5); // Balanced three-phase input
        clarke.calculate();
        let (alpha, beta, zero) = clarke.get_components();
        assert!((alpha - 1.0).abs() < 1e-6, "Alpha should be 1.0");
        assert!(beta.abs() < 1e-6, "Beta should be 0.0");
        assert!(zero.abs() < 1e-6, "Zero should be 0.0");
    }

    #[test]
    fn test_clarke_transform_unbalanced_inputs() {
        let mut clarke = Clarke::new();
        clarke.set_input(2.0, -1.0, -0.5); // Unbalanced input
        clarke.calculate();
        let (alpha, beta, zero) = clarke.get_components();
        assert!((alpha - 1.5).abs() < 1e-6, "Alpha should be 1.5");
        assert!(
            (beta + 0.577_350_3).abs() < 1e-6,
            "Beta should be approximately -0.577"
        );
        assert!(
            (zero - 0.166_666_7).abs() < 1e-6,
            "Zero should be approximately 0.167"
        );
    }

    #[test]
    fn test_clarke_transform_zero_inputs() {
        let mut clarke = Clarke::new();
        clarke.set_input(0.0, 0.0, 0.0); // All inputs are zero
        clarke.calculate();
        let (alpha, beta, zero) = clarke.get_components();
        assert!(alpha.abs() < 1e-6, "Alpha should be 0.0");
        assert!(beta.abs() < 1e-6, "Beta should be 0.0");
        assert!(zero.abs() < 1e-6, "Zero should be 0.0");
    }

    #[test]
    fn test_clarke_transform_positive_inputs() {
        let mut clarke = Clarke::new();
        clarke.set_input(1.0, 1.0, 1.0); // All inputs are equal
        clarke.calculate();
        let (alpha, beta, zero) = clarke.get_components();
        assert!(alpha.abs() < 1e-6, "Alpha should be 0.0");
        assert!(beta.abs() < 1e-6, "Beta should be 0.0");
        assert!((zero - 1.0).abs() < 1e-6, "Zero should be 1.0");
    }

    #[test]
    fn test_clarke_transform_negative_inputs() {
        let mut clarke = Clarke::new();
        clarke.set_input(-1.0, -1.0, -1.0); // All inputs are equal and negative
        clarke.calculate();
        let (alpha, beta, zero) = clarke.get_components();
        assert!(alpha.abs() < 1e-6, "Alpha should be 0.0");
        assert!(beta.abs() < 1e-6, "Beta should be 0.0");
        assert!((zero + 1.0).abs() < 1e-6, "Zero should be -1.0");
    }

    #[test]
    fn test_clarke_transform_single_phase_active() {
        let mut clarke = Clarke::new();
        clarke.set_input(1.0, 0.0, 0.0); // Only phase A is active
        clarke.calculate();
        let (alpha, beta, zero) = clarke.get_components();
        assert!(
            (alpha - 0.666_666_7).abs() < 1e-6,
            "Alpha should be approximately 0.667"
        );
        assert!(
            (beta - 0.577_350_3).abs() < 1e-6,
            "Beta should be approximately 0.577"
        );
        assert!(
            (zero - 0.333_333_3).abs() < 1e-6,
            "Zero should be approximately 0.333"
        );
    }

    #[test]
    fn test_clarke_transform_phase_shifted() {
        let mut clarke = Clarke::new();
        clarke.set_input(0.0, 1.0, -1.0); // 120° phase shift
        clarke.calculate();
        let (alpha, beta, zero) = clarke.get_components();
        assert!(
            (alpha + 0.666_666_7).abs() < 1e-6,
            "Alpha should be approximately -0.667"
        );
        assert!(
            (beta - 1.154_700_5).abs() < 1e-6,
            "Beta should be approximately 1.155"
        );
        assert!(zero.abs() < 1e-6, "Zero should be 0.0");
    }

    #[test]
    fn test_clarke_transform_large_inputs() {
        let mut clarke = Clarke::new();
        clarke.set_input(100.0, 50.0, -150.0); // Large unbalanced inputs
        clarke.calculate();
        let (alpha, beta, zero) = clarke.get_components();
        assert!(
            (alpha - 116.666_664).abs() < 1e-6,
            "Alpha should be approximately 116.667"
        );
        assert!(
            (beta + 230.940_11).abs() < 1e-6,
            "Beta should be approximately -230.94"
        );
        assert!(
            (zero - 0.0).abs() < 1e-6,
            "Zero should be approximately 0.0"
        );
    }
}
