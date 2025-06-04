#[cfg(test)]
mod tests {
    extern crate std;
    use libm::{fabsf, sinf};
    use libpower::filter::chebyshev_lpf::*;

    #[test]
    fn test_filter_creation() {
        let mut filter = ChebyshevLPF::<4>::new_uninit();
        assert!(filter.init(4, 0.1, 44100.0, 1000.0).is_ok());
    }

    #[test]
    fn test_invalid_parameters() {
        let mut filter = ChebyshevLPF::<4>::new_uninit();

        // Test odd order
        assert!(filter.init(3, 0.1, 44100.0, 1000.0).is_err());

        // Test invalid epsilon
        assert!(filter.init(4, 0.0, 44100.0, 1000.0).is_err());

        // Test invalid frequency
        assert!(filter.init(4, 0.1, 44100.0, 25000.0).is_err());
    }

    #[test]
    fn test_nyquist_rejection() {
        const PI: f32 = core::f32::consts::PI;
        let mut filter = ChebyshevLPF::<4>::new_uninit();
        filter.init(4, 0.1, 44100.0, 1000.0).unwrap();

        let mut max_output = 0.0f32;

        // Test with Nyquist frequency input
        for i in 0..1000 {
            let input = sinf(i as f32 * PI);
            let output = filter.process(input);
            max_output = if fabsf(output) > max_output {
                fabsf(output)
            } else {
                max_output
            };
        }

        assert!(max_output < 0.1, "Nyquist rejection error: {}", max_output);
    }

    #[test]
    fn test_reset() {
        let mut filter = ChebyshevLPF::<4>::new_uninit();
        filter.init(4, 0.1, 44100.0, 1000.0).unwrap();

        // Process some samples
        for _ in 0..100 {
            filter.process(1.0);
        }

        // Reset filter
        filter.reset();

        // Check if all state variables are zeroed
        for i in 0..filter.get_m() {
            assert_eq!(filter.get_w0()[i], 0.0);
            assert_eq!(filter.get_w1()[i], 0.0);
            assert_eq!(filter.get_w2()[i], 0.0);
        }
    }
}
