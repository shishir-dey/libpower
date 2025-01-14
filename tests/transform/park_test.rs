#[cfg(test)]
mod tests {
    use libpower::transform::park::*;

    #[test]
    fn test_park_transform_zero_angle() {
        let mut park = Park::new();
        park.set_input(1.0, 0.0, 0.0); // α = 1, β = 0, zero = 0
        park.set_angle(0.0); // Angle = 0 radians
        park.calculate();
        let (d, q, z) = park.get_components();
        assert!((d - 1.0).abs() < 1e-6, "D should be 1.0");
        assert!(q.abs() < 1e-6, "Q should be 0.0");
        assert!(z.abs() < 1e-6, "Z should be 0.0");
    }

    #[test]
    fn test_park_transform_90_deg_angle() {
        let mut park = Park::new();
        park.set_input(1.0, 0.0, 0.0); // α = 1, β = 0, zero = 0
        park.set_angle(core::f32::consts::FRAC_PI_2); // Angle = 90 degrees
        park.calculate();
        let (d, q, z) = park.get_components();
        assert!(d.abs() < 1e-6, "D should be 0.0");
        assert!((q - (-1.0)).abs() < 1e-6, "Q should be -1.0");
        assert!(z.abs() < 1e-6, "Z should be 0.0");
    }

    #[test]
    fn test_park_transform_180_deg_angle() {
        let mut park = Park::new();
        park.set_input(1.0, 0.0, 0.0); // α = 1, β = 0, zero = 0
        park.set_angle(core::f32::consts::PI); // Angle = 180 degrees
        park.calculate();
        let (d, q, z) = park.get_components();
        assert!((d - (-1.0)).abs() < 1e-6, "D should be -1.0");
        assert!(q.abs() < 1e-6, "Q should be 0.0");
        assert!(z.abs() < 1e-6, "Z should be 0.0");
    }

    #[test]
    fn test_park_transform_arbitrary_angle() {
        let mut park = Park::new();
        park.set_input(1.0, 1.0, 0.0); // α = 1, β = 1, zero = 0
        park.set_angle(core::f32::consts::FRAC_PI_4); // Angle = 45 degrees
        park.calculate();
        let (d, q, z) = park.get_components();
        let sqrt_2_over_2 = (2.0_f32).sqrt() / 2.0;
        assert!((d - (2.0 * sqrt_2_over_2)).abs() < 1e-6, "D should be √2");
        assert!(q.abs() < 1e-6, "Q should be 0.0");
        assert!(z.abs() < 1e-6, "Z should be 0.0");
    }

    #[test]
    fn test_park_transform_negative_inputs() {
        let mut park = Park::new();
        park.set_input(-1.0, -1.0, 0.0); // α = -1, β = -1, zero = 0
        park.set_angle(0.0); // Angle = 0 radians
        park.calculate();
        let (d, q, z) = park.get_components();
        assert!((d + 1.0).abs() < 1e-6, "D should be -1.0");
        assert!((q + 1.0).abs() < 1e-6, "Q should be -1.0");
        assert!(z.abs() < 1e-6, "Z should be 0.0");
    }

    #[test]
    fn test_park_transform_full_circle() {
        let mut park = Park::new();
        park.set_input(1.0, 0.0, 0.0); // α = 1, β = 0, zero = 0
        for i in 0..=360 {
            let angle = i as f32 * core::f32::consts::PI / 180.0; // Angle in radians
            park.set_angle(angle);
            park.calculate();
            let (d, q, _) = park.get_components();
            assert!(
                (d - angle.cos()).abs() < 1e-6,
                "D mismatch at angle: {}°",
                i
            );
            assert!(
                (q - (-angle.sin())).abs() < 1e-6,
                "Q mismatch at angle: {}°",
                i
            );
        }
    }

    #[test]
    fn test_park_transform_large_angles() {
        let mut park = Park::new();
        park.set_input(1.0, 0.0, 0.0); // α = 1, β = 0, zero = 0

        // Normalize the angle to within [0, 2π]
        let angle = 10.0 * core::f32::consts::PI; // 10π (5 full rotations)
        let normalized_angle = angle % (2.0 * core::f32::consts::PI);

        park.set_angle(normalized_angle); // Use normalized angle
        park.calculate();

        let (d, q, z) = park.get_components();
        assert!((d - 1.0).abs() < 1e-6, "D should be 1.0, got {}", d);
        assert!(q.abs() < 1e-6, "Q should be 0.0, got {}", q);
        assert!(z.abs() < 1e-6, "Z should be 0.0, got {}", z);
    }
}
