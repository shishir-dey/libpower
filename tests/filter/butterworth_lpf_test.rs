#[cfg(test)]
mod tests {
    use core::f32::consts::PI;
    use libm::sinf;
    use libpower::filter::butterworth_lpf::*;

    #[test]
    fn test_filter_creation() {
        let mut filter = ButterworthLPF::<4>::new_uninit();
        filter.init(4, 1000.0, 44100.0);
        assert_eq!(filter.get_order(), 4);
    }

    #[test]
    fn test_order_bounds() {
        let mut filter_low = ButterworthLPF::<4>::new_uninit();
        filter_low.init(1, 1000.0, 44100.0);
        assert_eq!(filter_low.get_order(), 2); // Minimum order is 2

        let mut filter_high = ButterworthLPF::<4>::new_uninit();
        filter_high.init(10, 1000.0, 44100.0);
        assert_eq!(filter_high.get_order(), 8); // Maximum order is 8
    }

    #[test]
    fn test_reset() {
        let mut filter = ButterworthLPF::<4>::new_uninit();
        filter.init(4, 1000.0, 44100.0);

        // Process some samples
        filter.process(1.0);
        filter.process(2.0);

        filter.reset();

        // Verify all states are zero
        for i in 0..filter.get_n() {
            assert_eq!(filter.get_w0()[i], 0.0);
            assert_eq!(filter.get_w1()[i], 0.0);
            assert_eq!(filter.get_w2()[i], 0.0);
        }
    }

    #[test]
    fn test_passband() {
        let mut filter = ButterworthLPF::<4>::new_uninit();
        let fc = 1000.0;
        let fs = 44100.0;
        filter.init(4, fc, fs);

        // Test with frequency well below cutoff (500Hz)
        let freq = 500.0;
        let mut max_output = 0.0f32;

        // Let the filter settle
        for _ in 0..100 {
            filter.process(0.0);
        }

        // Test with low-frequency sine wave
        for i in 0..1000 {
            let t = i as f32 / fs;
            let input = sinf(2.0 * PI * freq * t);
            let output = filter.process(input);
            max_output = max_output.max(output.abs());
        }

        // Low frequencies should pass through with minimal attenuation
        assert!(max_output > 0.9);
    }

    #[test]
    fn test_all_orders() {
        let fs = 44100.0;
        let fc = 1000.0;

        for order in (2..=8).step_by(2) {
            let mut filter = ButterworthLPF::<4>::new_uninit();
            filter.init(order, fc, fs);

            // Verify order was set correctly
            assert_eq!(filter.get_order(), order);

            // Test basic functionality
            let output = filter.process(1.0);
            assert!(!output.is_nan());
            assert!(!output.is_infinite());
        }
    }
}
