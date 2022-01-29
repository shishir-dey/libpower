#[allow(dead_code)]
enum SignalType {
    DC,
    Sine,
    FullWaveRectifiedSine,
    HalfWaveRectifiedSine,
    PWM,
    Triangular,
    Square,
    Sawtooth,
    GaussianNoise,
}

#[allow(dead_code)]
struct Signal {
    wave_type: SignalType,
    amplitude: f32,
    phase: f32,
    frequency: f32,
    num_samples: u32,
    /* internal */
    peak_to_peak: f32,
    max: f32,
    min: f32,
    average: f32,
    dc_rms: f32,
    ac_rms: f32,
    duty_cycle_pos: f32,
    duty_cycle_neg: f32,
    rise_time: f32,
    fall_time: f32,
    crest_factor: f32,
    thd: f32,
}

impl Signal {
    #[allow(dead_code)]
    pub fn new(wave_type: SignalType, amplitude: f32, frequency: f32, num_samples: u32) -> Signal {
        Signal {
            wave_type: wave_type,
            amplitude: amplitude,
            phase: 0.0,
            frequency: frequency,
            num_samples: num_samples,
            peak_to_peak: 0.0,
            max: 0.0,
            min: 0.0,
            average: 0.0,
            dc_rms: 0.0,
            ac_rms: 0.0,
            duty_cycle_pos: 0.0,
            duty_cycle_neg: 0.0,
            rise_time: 0.0,
            fall_time: 0.0,
            crest_factor: 0.0,
            thd: 0.0,
        }
    }
}

/* Placeholder for module's unit tests */
#[cfg(test)]
#[allow(unused_imports)]
mod tests {
    use super::*;

    #[test]
    fn test_will_always_fail() {
        assert!(false);
    }
}
