use core::cmp::{max, min};
use libm::{sinf, tanf};

#[derive(Debug)]
pub struct ButterworthLPF<const N: usize> {
    order: usize,
    n: usize,     // Number of second-order sections
    wc: f32,      // Angular cutoff frequency
    t: f32,       // Sample period
    a: [f32; N],  // Gain coefficients
    d1: [f32; N], // First feedback coefficients
    d2: [f32; N], // Second feedback coefficients
    w0: [f32; N], // Current state
    w1: [f32; N], // First delayed state
    w2: [f32; N], // Second delayed state
}

impl<const N: usize> ButterworthLPF<N> {
    /// Create an uninitialized filter
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

    /// Get the filter order
    pub fn get_order(&self) -> usize {
        self.order
    }

    /// Get the number of second-order sections
    pub fn get_n(&self) -> usize {
        self.n
    }

    /// Get the current state
    pub fn get_w0(&self) -> &[f32] {
        &self.w0[..self.n]
    }

    /// Get the first delayed state
    pub fn get_w1(&self) -> &[f32] {
        &self.w1[..self.n]
    }

    /// Get the second delayed state
    pub fn get_w2(&self) -> &[f32] {
        &self.w2[..self.n]
    }

    /// Initialize the filter with the given order, cutoff frequency, and sampling frequency
    pub fn init(&mut self, order: usize, fc: f32, fs: f32) {
        const PI: f32 = core::f32::consts::PI;

        // Bound the order between 2 and 8 (n between 1 and 4)
        self.order = min(max(order, 2), 8);
        self.order = self.order - (self.order % 2); // Ensure order is even
        self.n = self.order / 2;

        self.t = 1.0 / fs;

        // Pre-warp cutoff frequency (Tustin transformation)
        let a = tanf(PI * fc * self.t / 2.0);
        let a2 = a * a;

        // Calculate coefficients for each section
        for i in 0..self.n {
            let r = sinf(PI * (2.0 * i as f32 + 1.0) / (4.0 * self.n as f32));
            let s = a2 + 2.0 * a * r + 1.0;

            self.a[i] = a2 / s;
            self.d1[i] = 2.0 * (1.0 - a2) / s;
            self.d2[i] = -(a2 - 2.0 * a * r + 1.0) / s;
        }
    }

    /// Reset the filter states
    pub fn reset(&mut self) {
        for i in 0..self.n {
            self.w0[i] = 0.0;
            self.w1[i] = 0.0;
            self.w2[i] = 0.0;
        }
    }

    /// Process a single input sample
    pub fn process(&mut self, input: f32) -> f32 {
        let mut x = input;

        // Process through each second-order section
        for i in 0..self.n {
            // Save previous states
            self.w2[i] = self.w1[i];
            self.w1[i] = self.w0[i];

            // Process current input
            self.w0[i] = self.a[i] * (x + 2.0 * self.w1[i] + self.w2[i])
                - self.d1[i] * self.w1[i]
                - self.d2[i] * self.w2[i];

            x = self.w0[i]; // Output becomes input for the next section
        }

        x
    }
}
