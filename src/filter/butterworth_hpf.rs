use core::cmp::{max, min};
use libm::{sinf, tanf};

#[derive(Debug)]
pub struct ButterworthHPF<const N: usize> {
    order: usize,
    n: usize,     // n = order/2 for processing
    wc: f32,      // Angular cutoff frequency
    t: f32,       // Sample period
    a: [f32; N],  // Gain coefficients
    d1: [f32; N], // First delay coefficients
    d2: [f32; N], // Second delay coefficients
    w0: [f32; N], // Current state
    w1: [f32; N], // First delayed state
    w2: [f32; N], // Second delayed state
}

impl<const N: usize> ButterworthHPF<N> {
    pub const fn new_uninit() -> Self {
        Self {
            order: 0,
            n: 0,
            wc: 0.0,
            t: 0.0,
            a: [0.0; N],
            d1: [0.0; N],
            d2: [0.0; N],
            w0: [0.0; N],
            w1: [0.0; N],
            w2: [0.0; N],
        }
    }

    pub fn get_order(&self) -> usize {
        self.order
    }

    pub fn get_n(&self) -> usize {
        self.n
    }

    pub fn get_w0(&self) -> &[f32] {
        &self.w0[..self.n]
    }

    pub fn get_w1(&self) -> &[f32] {
        &self.w1[..self.n]
    }

    pub fn get_w2(&self) -> &[f32] {
        &self.w2[..self.n]
    }

    pub fn init(&mut self, order: usize, fc: f32, fs: f32) {
        const PI: f32 = core::f32::consts::PI;

        // Bound the order between 2 and 8 (n between 1 and 4)
        self.order = min(max(order, 2), 8);
        // Ensure order is even
        self.order = self.order - (self.order % 2);
        self.n = self.order / 2;

        self.t = 1.0 / fs;

        // Pre-warp cutoff frequency (Tustin)
        let a = tanf(PI * fc * self.t / 2.0);
        let a2 = a * a;

        // Calculate coefficients for each section
        for i in 0..self.n {
            let r = sinf(PI * (2.0 * i as f32 + 1.0) / (4.0 * self.n as f32));
            let s = a2 + 2.0 * a * r + 1.0;

            self.a[i] = 1.0 / s;
            self.d1[i] = 2.0 * (1.0 - a2) / s;
            self.d2[i] = -(a2 - 2.0 * a * r + 1.0) / s;
        }
    }

    pub fn reset(&mut self) {
        for i in 0..self.n {
            self.w0[i] = 0.0;
            self.w1[i] = 0.0;
            self.w2[i] = 0.0;
        }
    }

    pub fn process(&mut self, input: f32) -> f32 {
        let mut x = input;

        // Process through each second-order section
        for i in 0..self.n {
            // Save previous states
            self.w2[i] = self.w1[i];
            self.w1[i] = self.w0[i];

            // Process current input
            self.w0[i] = self.a[i] * (x - 2.0 * self.w1[i] + self.w2[i])
                - self.d1[i] * self.w1[i]
                - self.d2[i] * self.w2[i];

            x = self.w0[i]; // Output becomes input for next section
        }

        x
    }
}
