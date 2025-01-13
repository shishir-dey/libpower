#[cfg(test)]
mod tests {
    use libpower::signal::signal::*;

    const EPSILON: f32 = 1e-6;

    #[test]
    fn test_dc_signal() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::DC, 2.0, 0.0, &mut buffer);
        let metrics = signal.get_metrics();

        assert!((metrics.max - 2.0).abs() < EPSILON);
        assert!((metrics.min - 2.0).abs() < EPSILON);
        assert!((metrics.average - 2.0).abs() < EPSILON);
        assert!(metrics.peak_to_peak.abs() < EPSILON);
        assert!((metrics.dc_rms - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_sine_wave_properties() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Sine, 1.0, 1.0, &mut buffer);
        let metrics = signal.get_metrics();

        assert!((metrics.peak_to_peak - 2.0).abs() < 0.1);
        assert!(metrics.average.abs() < 0.1);
        assert!((metrics.dc_rms - 0.707).abs() < 0.1);
    }

    #[test]
    fn test_pwm_signal() {
        let mut buffer = [0.0f32; 1000];
        let mut signal = Signal::new(SignalType::PWM, 1.0, 1.0, &mut buffer);

        signal.set_duty_cycle(0.25);
        let metrics = signal.get_metrics();
        assert!((metrics.duty_cycle_pos - 0.25).abs() < 0.15);

        signal.set_duty_cycle(0.75);
        let metrics = signal.get_metrics();
        assert!((metrics.duty_cycle_pos - 0.75).abs() < 0.15);
    }

    #[test]
    fn test_triangular_wave() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Triangular, 1.0, 1.0, &mut buffer);
        let metrics = signal.get_metrics();

        assert!((metrics.peak_to_peak - 2.0).abs() < 0.1);
        assert!(metrics.average.abs() < 0.1);
    }

    #[test]
    fn test_full_wave_rectified_sine() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::FullWaveRectifiedSine, 1.0, 1.0, &mut buffer);
        let metrics = signal.get_metrics();

        assert!(metrics.min >= -EPSILON);
        assert!((metrics.max - 1.0).abs() < 0.1);
        assert!(metrics.average > 0.0);
    }
}
