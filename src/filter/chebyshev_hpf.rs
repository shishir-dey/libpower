use libm::{cosf, coshf, logf, sinf, sinhf, sqrtf, tanf};

#[derive(Debug)]
pub struct ChebyshevHPF<const N: usize> {
    m: usize,     // N/2 - number of filter sections
    a: [f32; N],  // Gain coefficients
    d1: [f32; N], // First delay coefficients
    d2: [f32; N], // Second delay coefficients
    w0: [f32; N], // State variables
    w1: [f32; N],
    w2: [f32; N],
    ep: f32, // epsilon normalization factor
}

impl<const N: usize> ChebyshevHPF<N> {
    pub const fn new_uninit() -> Self {
        Self {
            m: 0,
            a: [0.0; N],
            d1: [0.0; N],
            d2: [0.0; N],
            w0: [0.0; N],
            w1: [0.0; N],
            w2: [0.0; N],
            ep: 0.0,
        }
    }

    pub fn get_m(&self) -> usize {
        self.m
    }

    pub fn get_w0(&self) -> &[f32] {
        &self.w0[..self.m]
    }

    pub fn get_w1(&self) -> &[f32] {
        &self.w1[..self.m]
    }

    pub fn get_w2(&self) -> &[f32] {
        &self.w2[..self.m]
    }

    pub fn init(
        &mut self,
        order: usize,
        epsilon: f32,
        sample_rate: f32,
        cutoff_freq: f32,
    ) -> Result<(), &'static str> {
        if order % 2 != 0 {
            return Err("Order must be even");
        }
        if order > N * 2 {
            return Err("Order too large for allocated size");
        }
        if epsilon <= 0.0 {
            return Err("Epsilon must be positive");
        }
        if cutoff_freq <= 0.0 || cutoff_freq >= sample_rate / 2.0 {
            return Err("Invalid cutoff frequency");
        }

        self.m = order / 2;
        const PI: f32 = core::f32::consts::PI;

        // Prewarp cutoff frequency
        let a = tanf(PI * cutoff_freq / sample_rate);
        let a2 = a * a;

        // Calculate filter parameters
        let u = logf(1.0 + sqrtf(1.0 + epsilon * epsilon) / epsilon);
        let su = sinhf(u / order as f32);
        let cu = coshf(u / order as f32);

        // Calculate coefficients for each section
        for i in 0..self.m {
            let b = sinf(PI * (2.0 * i as f32 + 1.0) / (2.0 * order as f32)) * su;
            let c = cosf(PI * (2.0 * i as f32 + 1.0) / (2.0 * order as f32)) * cu;
            let c = b * b + c * c;

            // High-pass specific coefficient calculations
            let s = a2 + 2.0 * a * b + c;
            self.a[i] = 1.0 / (4.0 * s);
            self.d1[i] = 2.0 * (c - a2) / s;
            self.d2[i] = -(a2 - 2.0 * a * b + c) / s;
        }

        self.ep = 2.0 / epsilon;
        self.reset();

        Ok(())
    }

    pub fn reset(&mut self) {
        for i in 0..self.m {
            self.w0[i] = 0.0;
            self.w1[i] = 0.0;
            self.w2[i] = 0.0;
        }
    }

    pub fn process(&mut self, input: f32) -> f32 {
        let mut output = input;

        // Process through each second-order section
        for i in 0..self.m {
            self.w0[i] = self.d1[i] * self.w1[i] + self.d2[i] * self.w2[i] + output;
            output = self.a[i] * (self.w0[i] - 2.0 * self.w1[i] + self.w2[i]); // Note the sign change for high-pass
            self.w2[i] = self.w1[i];
            self.w1[i] = self.w0[i];
        }

        output * self.ep
    }
}
