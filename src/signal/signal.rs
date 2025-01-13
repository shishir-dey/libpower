use core::f32::consts::PI;

extern crate libm; // For floating point math operations in no_std

#[derive(Debug, Clone, PartialEq)]
pub enum SignalType {
    DC,
    Sine,
    FullWaveRectifiedSine,
    HalfWaveRectifiedSine,
    PWM,
    Triangular,
    Square,
    Sawtooth,
}

#[derive(Debug)]
pub struct Signal<'a> {
    wave_type: SignalType,
    amplitude: f32,
    phase: f32,
    frequency: f32,
    num_samples: u32,
    duty_cycle: f32,
    // Internal measurements
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
    samples: &'a mut [f32], // Use slice instead of Vec for no_std
}

#[derive(Debug, Clone, Copy)]
pub struct SignalMetrics {
    pub peak_to_peak: f32,
    pub max: f32,
    pub min: f32,
    pub average: f32,
    pub dc_rms: f32,
    pub ac_rms: f32,
    pub duty_cycle_pos: f32,
    pub duty_cycle_neg: f32,
    pub rise_time: f32,
    pub fall_time: f32,
    pub crest_factor: f32,
    pub thd: f32,
}

impl<'a> Signal<'a> {
    pub fn new(
        wave_type: SignalType,
        amplitude: f32,
        frequency: f32,
        samples_buffer: &'a mut [f32],
    ) -> Signal<'a> {
        let num_samples = samples_buffer.len() as u32;
        let mut signal = Signal {
            wave_type,
            amplitude,
            phase: 0.0,
            frequency,
            num_samples,
            duty_cycle: 0.5,
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
            samples: samples_buffer,
        };
        signal.generate_samples();
        signal.calculate_metrics();
        signal
    }

    pub fn set_phase(&mut self, phase: f32) {
        self.phase = phase;
        self.generate_samples();
        self.calculate_metrics();
    }

    pub fn set_duty_cycle(&mut self, duty_cycle: f32) {
        self.duty_cycle = if duty_cycle < 0.0 {
            0.0
        } else if duty_cycle > 1.0 {
            1.0
        } else {
            duty_cycle
        };
        self.generate_samples();
        self.calculate_metrics();
    }

    pub fn get_samples(&self) -> &[f32] {
        self.samples
    }

    pub fn get_metrics(&self) -> SignalMetrics {
        SignalMetrics {
            peak_to_peak: self.peak_to_peak,
            max: self.max,
            min: self.min,
            average: self.average,
            dc_rms: self.dc_rms,
            ac_rms: self.ac_rms,
            duty_cycle_pos: self.duty_cycle_pos,
            duty_cycle_neg: self.duty_cycle_neg,
            rise_time: self.rise_time,
            fall_time: self.fall_time,
            crest_factor: self.crest_factor,
            thd: self.thd,
        }
    }

    fn generate_samples(&mut self) {
        let sample_rate = self.num_samples as f32;

        for i in 0..self.num_samples {
            let t = i as f32 / sample_rate;
            let angular_freq = 2.0 * PI * self.frequency;
            let phase_rad = self.phase * PI / 180.0;

            let sample = match self.wave_type {
                SignalType::DC => self.amplitude,

                SignalType::Sine => self.amplitude * libm::sinf(angular_freq * t + phase_rad),

                SignalType::FullWaveRectifiedSine => {
                    self.amplitude * libm::fabsf(libm::sinf(angular_freq * t + phase_rad))
                }

                SignalType::HalfWaveRectifiedSine => {
                    let val = self.amplitude * libm::sinf(angular_freq * t + phase_rad);
                    if val > 0.0 {
                        val
                    } else {
                        0.0
                    }
                }

                SignalType::PWM => {
                    let duty = libm::sinf(angular_freq * t + phase_rad);
                    if duty > (1.0 - 2.0 * self.duty_cycle) {
                        self.amplitude
                    } else {
                        -self.amplitude
                    }
                }

                SignalType::Triangular => {
                    let period = 1.0 / self.frequency;
                    let t_mod = (t + phase_rad / angular_freq) % period;
                    let normalized = t_mod / period;

                    if normalized < 0.5 {
                        self.amplitude * (4.0 * normalized - 1.0)
                    } else {
                        self.amplitude * (3.0 - 4.0 * normalized)
                    }
                }

                SignalType::Square => {
                    if libm::sinf(angular_freq * t + phase_rad) >= 0.0 {
                        self.amplitude
                    } else {
                        -self.amplitude
                    }
                }

                SignalType::Sawtooth => {
                    let period = 1.0 / self.frequency;
                    let t_mod = (t + phase_rad / angular_freq) % period;
                    self.amplitude * (2.0 * t_mod / period - 1.0)
                }
            };

            self.samples[i as usize] = sample;
        }
    }

    fn calculate_metrics(&mut self) {
        // Basic statistics
        self.max = f32::NEG_INFINITY;
        self.min = f32::INFINITY;
        let mut sum = 0.0;
        let mut squared_sum = 0.0;
        let mut positive_samples = 0;

        for &sample in self.samples.iter() {
            self.max = libm::fmaxf(self.max, sample);
            self.min = libm::fminf(self.min, sample);
            sum += sample;
            squared_sum += sample * sample;
            if sample > 0.0 {
                positive_samples += 1;
            }
        }

        self.peak_to_peak = self.max - self.min;
        self.average = sum / self.num_samples as f32;

        // RMS calculations
        self.dc_rms = libm::sqrtf(squared_sum / self.num_samples as f32);

        // AC RMS (remove DC component)
        let mut ac_squared_sum = 0.0;
        for &sample in self.samples.iter() {
            let ac_sample = sample - self.average;
            ac_squared_sum += ac_sample * ac_sample;
        }
        self.ac_rms = libm::sqrtf(ac_squared_sum / self.num_samples as f32);

        // Duty cycle calculations
        self.duty_cycle_pos = positive_samples as f32 / self.num_samples as f32;
        self.duty_cycle_neg = 1.0 - self.duty_cycle_pos;

        // Rise and fall time calculations (10% to 90%)
        let threshold_low = self.min + 0.1 * self.peak_to_peak;
        let threshold_high = self.min + 0.9 * self.peak_to_peak;

        let mut rise_samples = 0;
        let mut fall_samples = 0;
        let mut rising = false;
        let mut falling = false;

        for i in 1..self.samples.len() {
            let prev = self.samples[i - 1];
            let curr = self.samples[i];

            if prev <= threshold_low && curr > threshold_low {
                rising = true;
                falling = false;
                rise_samples = 0;
            }
            if rising && curr <= threshold_high {
                rise_samples += 1;
            }
            if prev >= threshold_high && curr < threshold_high {
                falling = true;
                rising = false;
                fall_samples = 0;
            }
            if falling && curr >= threshold_low {
                fall_samples += 1;
            }
        }

        let sample_period = 1.0 / (self.frequency * self.num_samples as f32);
        self.rise_time = rise_samples as f32 * sample_period;
        self.fall_time = fall_samples as f32 * sample_period;

        // Crest factor
        if self.dc_rms != 0.0 {
            self.crest_factor = self.peak_to_peak / (2.0 * self.dc_rms);
        }

        // Total Harmonic Distortion (simplified - only considers first 5 harmonics)
        if self.wave_type == SignalType::Sine {
            let fundamental = self.frequency;
            let mut harmonic_power = 0.0;

            for i in 2..=5 {
                let harmonic_freq = fundamental * i as f32;
                let harmonic_amplitude = self.compute_fft_magnitude(harmonic_freq);
                harmonic_power += harmonic_amplitude * harmonic_amplitude;
            }

            let fundamental_amplitude = self.compute_fft_magnitude(fundamental);
            if fundamental_amplitude != 0.0 {
                self.thd = (libm::sqrtf(harmonic_power) / fundamental_amplitude) * 100.0;
            }
        }
    }

    fn compute_fft_magnitude(&self, frequency: f32) -> f32 {
        let sample_rate = self.num_samples as f32;
        let mut real = 0.0;
        let mut imag = 0.0;

        for (i, &sample) in self.samples.iter().enumerate() {
            let t = i as f32 / sample_rate;
            let angle = 2.0 * PI * frequency * t;
            real += sample * libm::cosf(angle);
            imag += sample * libm::sinf(angle);
        }

        libm::sqrtf(real * real + imag * imag) / self.num_samples as f32
    }
}
