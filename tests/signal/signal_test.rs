#[cfg(test)]
mod tests {
    use libpower::signal::signal::*;

    const EPSILON: f32 = 1e-6;

    // =========================================================================
    // Existing tests (preserved & enhanced)
    // =========================================================================

    #[test]
    fn test_power_signal_constructor_and_instantaneous_power() {
        let mut buffer = [0.0f32; 32];
        let signal = Signal::new_power_port(230.0, 2.0, Some(50.0), &mut buffer);
        assert!((signal.instantaneous_power() - 460.0).abs() < EPSILON);
        assert_eq!(signal.frequency(), Some(50.0));
    }

    #[test]
    fn test_power_signal_harmonic_accessors() {
        let mut buffer = [0.0f32; 32];
        let mut signal = Signal::new_power_port(230.0, 2.0, Some(50.0), &mut buffer);
        signal.set_voltage_harmonic(3, Some(2.5));
        signal.set_current_harmonic(5, Some(1.5));

        assert_eq!(signal.voltage_harmonic(3), Some(2.5));
        assert_eq!(signal.current_harmonic(5), Some(1.5));
        assert_eq!(signal.voltage_harmonic(7), None);
    }

    #[test]
    fn test_ups_uses_signal_interface() {
        use libpower::system::ups::off_grid::UPS;

        let mut buffer = [0.0f32; 32];
        let mut mains = Signal::new_power_port(230.0, 2.0, Some(50.0), &mut buffer);
        mains.set_active_power(450.0);
        let ups = UPS::new(Some(&mains), None, None, None, None);
        let mains_interface = ups.mains().expect("mains interface should exist");

        assert!((mains_interface.active_power() - 450.0).abs() < EPSILON);
        assert_eq!(mains_interface.frequency(), Some(50.0));
    }

    // =========================================================================
    // DC signal — all metrics
    // =========================================================================

    #[test]
    fn test_dc_signal_all_metrics() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::DC, 5.0, 0.0, &mut buffer);
        let m = signal.get_metrics();

        assert!((m.max - 5.0).abs() < EPSILON);
        assert!((m.min - 5.0).abs() < EPSILON);
        assert!((m.average - 5.0).abs() < EPSILON);
        assert!(m.peak_to_peak.abs() < EPSILON);
        assert!((m.dc_rms - 5.0).abs() < EPSILON);
        assert!(m.ac_rms.abs() < EPSILON);
        assert!((m.positive_peak - 0.0).abs() < EPSILON);
        assert!((m.negative_peak - 0.0).abs() < EPSILON);
        // Crest factor for DC = 1.0 (peak == RMS)
        assert!((m.crest_factor - 1.0).abs() < 0.01);
        // THD should be 0 for DC
        assert!(m.thd.abs() < 0.1);
    }

    // =========================================================================
    // Sine wave — exhaustive metrics
    // =========================================================================

    #[test]
    fn test_sine_rms_and_average() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Sine, 1.0, 1.0, &mut buffer);
        let m = signal.get_metrics();

        // RMS of a sine = A / √2 ≈ 0.7071
        assert!((m.dc_rms - 0.7071).abs() < 0.02);
        // Average of a full sine ≈ 0
        assert!(m.average.abs() < 0.02);
        // Peak-to-peak = 2×A
        assert!((m.peak_to_peak - 2.0).abs() < 0.1);
    }

    #[test]
    fn test_sine_crest_factor() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Sine, 10.0, 1.0, &mut buffer);
        let m = signal.get_metrics();

        // Crest factor of sine = √2 ≈ 1.4142
        assert!((m.crest_factor - 1.4142).abs() < 0.05);
    }

    #[test]
    fn test_sine_form_factor() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Sine, 1.0, 1.0, &mut buffer);
        let m = signal.get_metrics();

        // Form factor of sine = π / (2√2) ≈ 1.1107
        assert!(
            (m.form_factor - 1.1107).abs() < 0.05,
            "Sine form factor = {}, expected ≈1.1107",
            m.form_factor
        );
    }

    #[test]
    fn test_sine_frequency_measurement() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Sine, 1.0, 5.0, &mut buffer);
        let m = signal.get_metrics();

        // Measured frequency should be close to the set frequency
        assert!(
            (m.frequency - 5.0).abs() < 1.0,
            "Measured freq = {}, expected ≈5.0",
            m.frequency
        );
        // Period = 1/f
        if m.frequency > 0.0 {
            assert!(
                (m.period - 0.2).abs() < 0.05,
                "Measured period = {}, expected ≈0.2",
                m.period
            );
        }
    }

    #[test]
    fn test_sine_thd_pure() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Sine, 1.0, 10.0, &mut buffer);
        let m = signal.get_metrics();

        // Pure sine should have THD ≈ 0%
        assert!(m.thd < 1.0, "Pure sine THD = {}%, expected <1%", m.thd);
    }

    #[test]
    fn test_sine_snr_pure() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Sine, 1.0, 10.0, &mut buffer);
        let m = signal.get_metrics();

        // Pure sine should have high SNR (>40 dB)
        assert!(
            m.snr > 40.0,
            "Pure sine SNR = {} dB, expected >40 dB",
            m.snr
        );
    }

    #[test]
    fn test_sine_phase_shift() {
        let mut buffer_0 = [0.0f32; 1000];
        let signal_0 = Signal::new(SignalType::Sine, 1.0, 5.0, &mut buffer_0);
        let m0 = signal_0.get_metrics();

        let mut buffer_90 = [0.0f32; 1000];
        let mut signal_90 = Signal::new(SignalType::Sine, 1.0, 5.0, &mut buffer_90);
        signal_90.set_phase(90.0);
        let m90 = signal_90.get_metrics();

        // RMS should be the same regardless of phase
        assert!(
            (m0.dc_rms - m90.dc_rms).abs() < 0.01,
            "RMS mismatch: {} vs {}",
            m0.dc_rms,
            m90.dc_rms
        );
    }

    // =========================================================================
    // Square wave
    // =========================================================================

    #[test]
    fn test_square_wave_metrics() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Square, 5.0, 1.0, &mut buffer);
        let m = signal.get_metrics();

        // Square wave: RMS = A, avg ≈ 0, crest ≈ 1.0
        assert!((m.dc_rms - 5.0).abs() < 0.1, "Square RMS = {}", m.dc_rms);
        assert!(m.average.abs() < 0.1, "Square avg = {}", m.average);
        assert!(
            (m.crest_factor - 1.0).abs() < 0.05,
            "Square crest = {}",
            m.crest_factor
        );
        assert!((m.max - 5.0).abs() < EPSILON);
        assert!((m.min - (-5.0)).abs() < EPSILON);
        assert!((m.peak_to_peak - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_square_wave_thd() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Square, 1.0, 10.0, &mut buffer);
        let m = signal.get_metrics();

        // Theoretical THD of square wave ≈ 48.3% (harmonics 3,5,7,...)
        // With limited harmonics the value may differ — accept 30–60%
        assert!(
            m.thd > 25.0 && m.thd < 70.0,
            "Square THD = {}%, expected 30–60%",
            m.thd
        );
    }

    // =========================================================================
    // Triangular wave
    // =========================================================================

    #[test]
    fn test_triangular_wave_metrics() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Triangular, 1.0, 1.0, &mut buffer);
        let m = signal.get_metrics();

        // Triangle: RMS = A / √3 ≈ 0.5774, avg ≈ 0
        assert!((m.dc_rms - 0.5774).abs() < 0.05, "Tri RMS = {}", m.dc_rms);
        assert!(m.average.abs() < 0.05, "Tri avg = {}", m.average);
        assert!((m.peak_to_peak - 2.0).abs() < 0.1);
        // Crest factor = √3 ≈ 1.732
        assert!(
            (m.crest_factor - 1.732).abs() < 0.1,
            "Tri crest = {}",
            m.crest_factor
        );
    }

    // =========================================================================
    // Sawtooth wave
    // =========================================================================

    #[test]
    fn test_sawtooth_wave_metrics() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Sawtooth, 1.0, 1.0, &mut buffer);
        let m = signal.get_metrics();

        // Sawtooth: RMS = A / √3 ≈ 0.5774, avg ≈ 0
        assert!((m.dc_rms - 0.5774).abs() < 0.05, "Saw RMS = {}", m.dc_rms);
        assert!(m.average.abs() < 0.05, "Saw avg = {}", m.average);
    }

    // =========================================================================
    // Full-wave rectified sine
    // =========================================================================

    #[test]
    fn test_full_wave_rectified_sine() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::FullWaveRectifiedSine, 1.0, 1.0, &mut buffer);
        let m = signal.get_metrics();

        // All values ≥ 0
        assert!(m.min >= -EPSILON, "FWR min = {}", m.min);
        assert!((m.max - 1.0).abs() < 0.02);
        // Average = 2A/π ≈ 0.6366
        assert!((m.average - 0.6366).abs() < 0.05, "FWR avg = {}", m.average);
        // RMS = A/√2 ≈ 0.7071 (same as original sine)
        assert!((m.dc_rms - 0.7071).abs() < 0.02, "FWR RMS = {}", m.dc_rms);
    }

    // =========================================================================
    // Half-wave rectified sine
    // =========================================================================

    #[test]
    fn test_half_wave_rectified_sine() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::HalfWaveRectifiedSine, 1.0, 1.0, &mut buffer);
        let m = signal.get_metrics();

        // All values ≥ 0
        assert!(m.min >= -EPSILON, "HWR min = {}", m.min);
        // Average = A/π ≈ 0.3183
        assert!((m.average - 0.3183).abs() < 0.05, "HWR avg = {}", m.average);
        // RMS = A/2 = 0.5
        assert!((m.dc_rms - 0.5).abs() < 0.05, "HWR RMS = {}", m.dc_rms);
    }

    // =========================================================================
    // PWM signal
    // =========================================================================

    #[test]
    fn test_pwm_duty_cycle_sweep() {
        let duties = [0.1f32, 0.25, 0.5, 0.75, 0.9];
        for &d in &duties {
            let mut buffer = [0.0f32; 1000];
            let mut signal = Signal::new(SignalType::PWM, 1.0, 1.0, &mut buffer);
            signal.set_duty_cycle(d);
            let m = signal.get_metrics();
            assert!(
                (m.duty_cycle_pos - d).abs() < 0.05,
                "PWM duty {}%: measured {}",
                d * 100.0,
                m.duty_cycle_pos
            );
        }
    }

    #[test]
    fn test_pwm_average_voltage() {
        let mut buffer = [0.0f32; 1000];
        let mut signal = Signal::new(SignalType::PWM, 5.0, 1.0, &mut buffer);
        signal.set_duty_cycle(0.4);
        let m = signal.get_metrics();

        // Average = A × duty = 5 × 0.4 = 2.0
        assert!(
            (m.average - 2.0).abs() < 0.3,
            "PWM avg = {}, expected 2.0",
            m.average
        );
    }

    #[test]
    fn test_pwm_rms() {
        let mut buffer = [0.0f32; 1000];
        let mut signal = Signal::new(SignalType::PWM, 5.0, 1.0, &mut buffer);
        signal.set_duty_cycle(0.5);
        let m = signal.get_metrics();

        // PWM RMS = A × √duty = 5 × √0.5 ≈ 3.536
        assert!(
            (m.dc_rms - 3.536).abs() < 0.3,
            "PWM RMS = {}, expected ≈3.536",
            m.dc_rms
        );
    }

    // =========================================================================
    // Edge cases
    // =========================================================================

    #[test]
    fn test_zero_amplitude() {
        let mut buffer = [0.0f32; 100];
        let signal = Signal::new(SignalType::Sine, 0.0, 1.0, &mut buffer);
        let m = signal.get_metrics();

        assert!((m.max).abs() < EPSILON);
        assert!((m.min).abs() < EPSILON);
        assert!((m.average).abs() < EPSILON);
        assert!((m.dc_rms).abs() < EPSILON);
        assert!((m.ac_rms).abs() < EPSILON);
        // Crest factor should be 0 (no div-by-zero)
        assert!((m.crest_factor).abs() < EPSILON);
    }

    #[test]
    fn test_single_sample_buffer() {
        let mut buffer = [0.0f32; 1];
        let signal = Signal::new(SignalType::DC, 3.0, 0.0, &mut buffer);
        let m = signal.get_metrics();

        assert!((m.max - 3.0).abs() < EPSILON);
        assert!((m.min - 3.0).abs() < EPSILON);
        assert!((m.average - 3.0).abs() < EPSILON);
    }

    #[test]
    fn test_negative_dc() {
        let mut buffer = [0.0f32; 100];
        let signal = Signal::new(SignalType::DC, -7.5, 0.0, &mut buffer);
        let m = signal.get_metrics();

        assert!((m.max - (-7.5)).abs() < EPSILON);
        assert!((m.min - (-7.5)).abs() < EPSILON);
        assert!((m.dc_rms - 7.5).abs() < EPSILON);
    }

    // =========================================================================
    // Standalone function tests — frequency measurement
    // =========================================================================

    #[test]
    fn test_measure_frequency_sine() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Sine, 1.0, 10.0, &mut buffer);
        let freq = measure_frequency(signal.get_samples(), 1000.0);

        assert!(
            (freq - 10.0).abs() < 1.0,
            "Measured freq = {}, expected ≈10.0",
            freq
        );
    }

    #[test]
    fn test_measure_frequency_dc_returns_zero() {
        let mut buffer = [0.0f32; 100];
        let signal = Signal::new(SignalType::DC, 5.0, 0.0, &mut buffer);
        let freq = measure_frequency(signal.get_samples(), 100.0);
        assert!(
            (freq).abs() < EPSILON,
            "DC should give freq=0, got {}",
            freq
        );
    }

    // =========================================================================
    // Standalone function tests — Goertzel
    // =========================================================================

    #[test]
    fn test_goertzel_single_tone() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Sine, 3.0, 50.0, &mut buffer);

        // Goertzel at the fundamental should return ≈ amplitude
        let mag = goertzel_magnitude(signal.get_samples(), 50.0, 1000.0);
        assert!(
            (mag - 3.0).abs() < 0.5,
            "Goertzel mag = {}, expected ≈3.0",
            mag
        );

        // Goertzel at a non-present frequency should be near zero
        let mag_off = goertzel_magnitude(signal.get_samples(), 120.0, 1000.0);
        assert!(
            mag_off < 0.5,
            "Goertzel off-freq mag = {}, expected ≈0",
            mag_off
        );
    }

    // =========================================================================
    // Standalone function tests — DFT spectrum
    // =========================================================================

    #[test]
    fn test_dft_magnitude_spectrum() {
        let mut buffer = [0.0f32; 256];
        let signal = Signal::new(SignalType::Sine, 2.0, 10.0, &mut buffer);

        let mut spectrum = [0.0f32; 128];
        dft_magnitude_spectrum(signal.get_samples(), &mut spectrum);

        // The fundamental bin should be the largest
        let fund_bin = 10; // freq * N / sample_rate = 10*256/256 = 10
        let fund_mag = spectrum[fund_bin];
        assert!(
            fund_mag > 1.0,
            "Fundamental mag = {}, expected >1.0",
            fund_mag
        );

        // DC should be near zero
        assert!(spectrum[0].abs() < 0.1, "DC bin = {}", spectrum[0]);
    }

    // =========================================================================
    // Standalone function tests — THD from spectrum
    // =========================================================================

    #[test]
    fn test_thd_from_spectrum_pure_sine() {
        let mut buffer = [0.0f32; 256];
        let signal = Signal::new(SignalType::Sine, 1.0, 10.0, &mut buffer);

        let mut spectrum = [0.0f32; 128];
        dft_magnitude_spectrum(signal.get_samples(), &mut spectrum);

        let thd = compute_thd_from_spectrum(&spectrum, 10);
        assert!(thd < 2.0, "Pure sine THD from spectrum = {}%", thd);
    }

    #[test]
    fn test_thd_from_spectrum_square() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Square, 1.0, 10.0, &mut buffer);

        let mut spectrum = [0.0f32; 500];
        dft_magnitude_spectrum(signal.get_samples(), &mut spectrum);

        let thd = compute_thd_from_spectrum(&spectrum, 10);
        // Square wave THD ≈ 48%
        assert!(
            thd > 25.0 && thd < 70.0,
            "Square THD from spectrum = {}%",
            thd
        );
    }

    // =========================================================================
    // Standalone function tests — SNR, SINAD, ENOB, SFDR
    // =========================================================================

    #[test]
    fn test_snr_sinad_pure_sine() {
        let mut buffer = [0.0f32; 1024];
        let signal = Signal::new(SignalType::Sine, 1.0, 20.0, &mut buffer);

        let mut spectrum = [0.0f32; 512];
        dft_magnitude_spectrum(signal.get_samples(), &mut spectrum);

        let snr = compute_snr(&spectrum, 20);
        let sinad = compute_sinad(&spectrum, 20);

        assert!(snr > 30.0, "SNR = {} dB", snr);
        assert!(sinad > 30.0, "SINAD = {} dB", sinad);
    }

    #[test]
    fn test_compute_enob() {
        // ENOB = (SINAD - 1.76) / 6.02
        let enob = compute_enob(50.0);
        let expected = (50.0 - 1.76) / 6.02;
        assert!(
            (enob - expected).abs() < EPSILON,
            "ENOB = {}, expected {}",
            enob,
            expected
        );
    }

    #[test]
    fn test_sfdr_pure_sine() {
        let mut buffer = [0.0f32; 1024];
        let signal = Signal::new(SignalType::Sine, 1.0, 20.0, &mut buffer);

        let mut spectrum = [0.0f32; 512];
        dft_magnitude_spectrum(signal.get_samples(), &mut spectrum);

        let sfdr = compute_sfdr(&spectrum, 20);
        // Pure sine should have high SFDR
        assert!(sfdr > 30.0, "SFDR = {} dB", sfdr);
    }

    // =========================================================================
    // Standalone function tests — windowing
    // =========================================================================

    #[test]
    fn test_windowing_hanning() {
        let mut buf = [1.0f32; 64];
        apply_window(&mut buf, WindowType::Hanning);

        // First and last samples should be near zero
        assert!(buf[0].abs() < 0.01, "Hanning buf[0] = {}", buf[0]);
        assert!(buf[63].abs() < 0.01, "Hanning buf[63] = {}", buf[63]);
        // Middle sample should be near 1.0
        assert!(
            (buf[32] - 1.0).abs() < 0.05,
            "Hanning buf[32] = {}",
            buf[32]
        );
    }

    #[test]
    fn test_windowing_hamming() {
        let mut buf = [1.0f32; 64];
        apply_window(&mut buf, WindowType::Hamming);

        // Hamming endpoints ≈ 0.08 (not zero like Hanning)
        assert!(buf[0] < 0.15, "Hamming buf[0] = {}", buf[0]);
        assert!(buf[63] < 0.15, "Hamming buf[63] = {}", buf[63]);
        // Middle should be near 1.0
        assert!(
            (buf[32] - 1.0).abs() < 0.05,
            "Hamming buf[32] = {}",
            buf[32]
        );
    }

    #[test]
    fn test_windowing_blackman() {
        let mut buf = [1.0f32; 64];
        apply_window(&mut buf, WindowType::Blackman);

        // Blackman endpoints should be near zero
        assert!(buf[0].abs() < 0.01, "Blackman buf[0] = {}", buf[0]);
        assert!(buf[63].abs() < 0.01, "Blackman buf[63] = {}", buf[63]);
    }

    #[test]
    fn test_windowing_rectangular() {
        let mut buf = [1.0f32; 64];
        apply_window(&mut buf, WindowType::Rectangular);
        // All samples should remain 1.0
        for (i, &s) in buf.iter().enumerate() {
            assert!((s - 1.0).abs() < EPSILON, "Rect buf[{}] = {}", i, s);
        }
    }

    // =========================================================================
    // Standalone function tests — signal arithmetic
    // =========================================================================

    #[test]
    fn test_add_dc_offset() {
        let mut buf = [1.0f32, 2.0, 3.0, 4.0];
        add_dc_offset(&mut buf, 10.0);
        assert!((buf[0] - 11.0).abs() < EPSILON);
        assert!((buf[3] - 14.0).abs() < EPSILON);
    }

    #[test]
    fn test_scale_signal() {
        let mut buf = [1.0f32, 2.0, 3.0, 4.0];
        scale(&mut buf, 0.5);
        assert!((buf[0] - 0.5).abs() < EPSILON);
        assert!((buf[3] - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_add_signals() {
        let a = [1.0f32, 2.0, 3.0];
        let b = [10.0f32, 20.0, 30.0];
        let mut out = [0.0f32; 3];
        add_signals(&a, &b, &mut out);

        assert!((out[0] - 11.0).abs() < EPSILON);
        assert!((out[1] - 22.0).abs() < EPSILON);
        assert!((out[2] - 33.0).abs() < EPSILON);
    }

    // =========================================================================
    // Standalone function tests — power calculation
    // =========================================================================

    #[test]
    fn test_power_calculation_resistive() {
        // In-phase V and I ⇒ PF = 1, Q = 0
        use core::f32::consts::PI;
        let n = 1000usize;
        let mut voltage = [0.0f32; 1000];
        let mut current = [0.0f32; 1000];

        let v_peak = 325.0; // ~230Vrms
        let i_peak = 14.14; // ~10Arms
        for i in 0..n {
            let t = i as f32 / n as f32;
            voltage[i] = v_peak * libm::sinf(2.0 * PI * t);
            current[i] = i_peak * libm::sinf(2.0 * PI * t);
        }

        let (p, q, _s, pf) = compute_power(&voltage, &current, n as f32);

        // P ≈ Vrms × Irms = 230 × 10 = 2300W
        assert!(
            (p - 2297.0).abs() < 50.0,
            "Active P = {}, expected ≈2297",
            p
        );
        // Q ≈ 0 for resistive load
        assert!(q < 50.0, "Reactive Q = {}, expected ≈0", q);
        // PF ≈ 1.0
        assert!((pf - 1.0).abs() < 0.02, "PF = {}, expected ≈1.0", pf);
    }

    #[test]
    fn test_power_calculation_reactive() {
        // 90° lagging current ⇒ P ≈ 0, PF ≈ 0
        use core::f32::consts::PI;
        let n = 1000usize;
        let mut voltage = [0.0f32; 1000];
        let mut current = [0.0f32; 1000];

        let v_peak = 325.0;
        let i_peak = 14.14;
        for i in 0..n {
            let t = i as f32 / n as f32;
            voltage[i] = v_peak * libm::sinf(2.0 * PI * t);
            current[i] = i_peak * libm::sinf(2.0 * PI * t - PI / 2.0); // 90° lag
        }

        let (p, q, _s, pf) = compute_power(&voltage, &current, n as f32);

        // Active power ≈ 0 for purely reactive load
        assert!(p.abs() < 50.0, "Active P = {}, expected ≈0", p);
        // Reactive power ≈ Vrms × Irms ≈ 2297
        assert!(q > 2000.0, "Reactive Q = {}, expected >2000", q);
        // PF ≈ 0
        assert!(pf.abs() < 0.05, "PF = {}, expected ≈0", pf);
    }

    // =========================================================================
    // Standalone function tests — phase difference
    // =========================================================================

    #[test]
    fn test_phase_difference_90deg() {
        use core::f32::consts::PI;
        let n = 1000usize;
        let mut sig_a = [0.0f32; 1000];
        let mut sig_b = [0.0f32; 1000];

        for i in 0..n {
            let t = i as f32 / n as f32;
            sig_a[i] = libm::sinf(2.0 * PI * 5.0 * t);
            sig_b[i] = libm::sinf(2.0 * PI * 5.0 * t + PI / 2.0); // +90°
        }

        let diff = phase_difference(&sig_a, &sig_b, n as f32);
        assert!(
            (diff - 90.0).abs() < 5.0,
            "Phase diff = {}°, expected ≈90°",
            diff
        );
    }

    #[test]
    fn test_phase_difference_zero() {
        use core::f32::consts::PI;
        let n = 1000usize;
        let mut sig_a = [0.0f32; 1000];
        let mut sig_b = [0.0f32; 1000];

        for i in 0..n {
            let t = i as f32 / n as f32;
            sig_a[i] = libm::sinf(2.0 * PI * 5.0 * t);
            sig_b[i] = libm::sinf(2.0 * PI * 5.0 * t); // same phase
        }

        let diff = phase_difference(&sig_a, &sig_b, n as f32);
        assert!(diff.abs() < 3.0, "Phase diff = {}°, expected ≈0°", diff);
    }

    // =========================================================================
    // Standalone function tests — slew rate
    // =========================================================================

    #[test]
    fn test_slew_rate_square_wave() {
        let mut buffer = [0.0f32; 1000];
        let signal = Signal::new(SignalType::Square, 5.0, 1.0, &mut buffer);
        let sr = slew_rate(signal.get_samples(), 1000.0);

        // Square wave has sharp transitions ⇒ high slew rate
        // dV = 10V in 1 sample ⇒ SR = 10 × 1000 = 10000 V/s
        assert!(sr > 5000.0, "Square slew rate = {}, expected >5000", sr);
    }

    #[test]
    fn test_slew_rate_dc() {
        let mut buffer = [0.0f32; 100];
        let signal = Signal::new(SignalType::DC, 5.0, 0.0, &mut buffer);
        let sr = slew_rate(signal.get_samples(), 100.0);

        // DC has zero slew rate
        assert!(sr.abs() < EPSILON, "DC slew rate = {}, expected 0", sr);
    }

    // =========================================================================
    // Reset and interface tests
    // =========================================================================

    #[test]
    fn test_reset_waveform() {
        let mut buffer = [0.0f32; 100];
        let mut signal = Signal::new(SignalType::Sine, 5.0, 10.0, &mut buffer);
        signal.reset_waveform();
        let m = signal.get_metrics();

        assert!((m.max).abs() < EPSILON);
        assert!((m.dc_rms).abs() < EPSILON);
        assert!((m.thd).abs() < EPSILON);
        assert!((m.frequency).abs() < EPSILON);
        assert!((m.snr).abs() < EPSILON);
    }

    #[test]
    fn test_reset_power_params() {
        let mut buffer = [0.0f32; 32];
        let mut signal = Signal::new_power_port(230.0, 10.0, Some(50.0), &mut buffer);
        signal.reset_power_params();

        assert!((signal.voltage()).abs() < EPSILON);
        assert!((signal.current()).abs() < EPSILON);
        assert_eq!(signal.frequency(), None);
        assert_eq!(signal.power_factor(), None);
    }

    #[test]
    fn test_set_limits() {
        let mut buffer = [0.0f32; 32];
        let mut signal = Signal::new_power_port(230.0, 10.0, Some(50.0), &mut buffer);
        signal.set_limits(180.0, 260.0, 20.0, 5000.0);

        assert!((signal.voltage_min() - 180.0).abs() < EPSILON);
        assert!((signal.voltage_max() - 260.0).abs() < EPSILON);
        assert!((signal.current_max() - 20.0).abs() < EPSILON);
        assert!((signal.power_max() - 5000.0).abs() < EPSILON);
    }

    #[test]
    fn test_state_and_temperature() {
        let mut buffer = [0.0f32; 32];
        let mut signal = Signal::new_power_port(230.0, 10.0, Some(50.0), &mut buffer);

        signal.set_state(false, true);
        assert!(!signal.is_enabled());
        assert!(signal.is_faulted());

        signal.set_temperature(Some(65.5));
        assert_eq!(signal.temperature(), Some(65.5));
    }
}
