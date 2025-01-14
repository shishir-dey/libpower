#[cfg(test)]
mod tests {
    use libpower::transform::ipark::*;

    #[test]
    fn test_inverse_park_transform_zero_angle() {
        let mut ipark = IPark::new();
        ipark.set_input(1.0, 0.0, 0.0); // d = 1, q = 0, zero = 0
        ipark.set_angle(0.0); // Angle = 0 radians
        ipark.calculate();
        let (alpha, beta, zero) = ipark.get_components();
        assert!((alpha - 1.0).abs() < 1e-6, "Alpha should be 1.0");
        assert!(beta.abs() < 1e-6, "Beta should be 0.0");
        assert!(zero.abs() < 1e-6, "Zero should be 0.0");
    }

    #[test]
    fn test_inverse_park_transform_90_deg_angle() {
        let mut ipark = IPark::new();
        ipark.set_input(1.0, 0.0, 0.0); // d = 1, q = 0, zero = 0
        ipark.set_angle(core::f32::consts::FRAC_PI_2); // Angle = 90 degrees
        ipark.calculate();
        let (alpha, beta, zero) = ipark.get_components();
        assert!(alpha.abs() < 1e-6, "Alpha should be 0.0");
        assert!((beta - 1.0).abs() < 1e-6, "Beta should be 1.0");
        assert!(zero.abs() < 1e-6, "Zero should be 0.0");
    }

    #[test]
    fn test_inverse_park_transform_180_deg_angle() {
        let mut ipark = IPark::new();
        ipark.set_input(1.0, 0.0, 0.0); // d = 1, q = 0, zero = 0
        ipark.set_angle(core::f32::consts::PI); // Angle = 180 degrees
        ipark.calculate();
        let (alpha, beta, zero) = ipark.get_components();
        assert!((alpha + 1.0).abs() < 1e-6, "Alpha should be -1.0");
        assert!(beta.abs() < 1e-6, "Beta should be 0.0");
        assert!(zero.abs() < 1e-6, "Zero should be 0.0");
    }

    #[test]
    fn test_inverse_park_transform_arbitrary_angle() {
        let mut ipark = IPark::new();
        ipark.set_input(1.0, 1.0, 0.0); // d = 1, q = 1, zero = 0
        ipark.set_angle(core::f32::consts::FRAC_PI_4); // Angle = 45 degrees
        ipark.calculate();
        let (alpha, beta, zero) = ipark.get_components();
        let sqrt_2_over_2 = (2.0_f32).sqrt() / 2.0;
        assert!((alpha - 0.0).abs() < 1e-6, "Alpha should be 0.0");
        assert!(
            (beta - (2.0 * sqrt_2_over_2)).abs() < 1e-6,
            "Beta should be √2"
        );
        assert!(zero.abs() < 1e-6, "Zero should be 0.0");
    }

    #[test]
    fn test_inverse_park_transform_negative_inputs() {
        let mut ipark = IPark::new();
        ipark.set_input(-1.0, -1.0, 0.0); // d = -1, q = -1, zero = 0
        ipark.set_angle(0.0); // Angle = 0 radians
        ipark.calculate();
        let (alpha, beta, zero) = ipark.get_components();
        assert!((alpha + 1.0).abs() < 1e-6, "Alpha should be -1.0");
        assert!((beta + 1.0).abs() < 1e-6, "Beta should be -1.0");
        assert!(zero.abs() < 1e-6, "Zero should be 0.0");
    }

    #[test]
    fn test_inverse_park_transform_full_circle() {
        let mut ipark = IPark::new();
        ipark.set_input(1.0, 0.0, 0.0); // d = 1, q = 0, zero = 0
        for i in 0..=360 {
            let angle = i as f32 * core::f32::consts::PI / 180.0; // Angle in radians
            ipark.set_angle(angle);
            ipark.calculate();
            let (alpha, beta, _) = ipark.get_components();
            assert!(
                (alpha - angle.cos()).abs() < 1e-6,
                "Alpha mismatch at angle: {}°",
                i
            );
            assert!(
                (beta - angle.sin()).abs() < 1e-6,
                "Beta mismatch at angle: {}°",
                i
            );
        }
    }

    #[test]
    fn test_inverse_park_transform_large_angles() {
        let mut ipark = IPark::new();
        ipark.set_input(1.0, 0.0, 0.0); // d = 1, q = 0, zero = 0

        // Normalize the angle to within [0, 2π]
        let angle = 10.0 * core::f32::consts::PI; // 10π (5 full rotations)
        let normalized_angle = angle % (2.0 * core::f32::consts::PI);

        ipark.set_angle(normalized_angle); // Use normalized angle
        ipark.calculate();

        let (alpha, beta, zero) = ipark.get_components();
        assert!(
            (alpha - 1.0).abs() < 1e-6,
            "Alpha should be 1.0, got {}",
            alpha
        );
        assert!(beta.abs() < 1e-6, "Beta should be 0.0, got {}", beta);
        assert!(zero.abs() < 1e-6, "Zero should be 0.0, got {}", zero);
    }
}
