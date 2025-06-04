#[cfg(test)]
mod tests {
    extern crate std;
    use libm::{fabsf, sinf};
    use libpower::filter::chebyshev_hpf::*;

    #[test]
    fn test_filter_creation() {
        let mut filter = ChebyshevHPF::<4>::new_uninit();
        assert!(filter.init(4, 0.1, 44100.0, 1000.0).is_ok());
    }

    #[test]
    fn test_invalid_parameters() {
        let mut filter = ChebyshevHPF::<4>::new_uninit();

        // Test odd order
        assert!(filter.init(3, 0.1, 44100.0, 1000.0).is_err());

        // Test invalid epsilon
        assert!(filter.init(4, 0.0, 44100.0, 1000.0).is_err());

        // Test invalid frequency
        assert!(filter.init(4, 0.1, 44100.0, 25000.0).is_err());
    }

    #[test]
    fn test_dc_rejection() {
        let mut filter = ChebyshevHPF::<4>::new_uninit();
        filter.init(4, 0.1, 44100.0, 1000.0).unwrap();

        // Feed DC signal and check if it's attenuated
        let mut output = 0.0;
        for _ in 0..1000 {
            output = filter.process(1.0);
        }

        // DC should be heavily attenuated
        assert!(fabsf(output) < 0.1, "DC rejection error: {}", output);
    }

    #[test]
    fn test_high_frequency_response() {
        const PI: f32 = core::f32::consts::PI;
        let mut filter = ChebyshevHPF::<4>::new_uninit();
        filter.init(4, 0.1, 44100.0, 1000.0).unwrap();

        let mut max_output = 0.0f32;

        // Test with high frequency input (well above cutoff)
        for i in 0..1000 {
            let input = sinf(i as f32 * PI * 0.8); // High frequency
            let output = filter.process(input);
            max_output = if fabsf(output) > max_output {
                fabsf(output)
            } else {
                max_output
            };
        }

        // High frequencies should pass through with less than 3dB attenuation
        assert!(
            max_output > 0.7,
            "High frequency response error: {}",
            max_output
        );
    }

    #[test]
    fn test_reset() {
        let mut filter = ChebyshevHPF::<4>::new_uninit();
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
