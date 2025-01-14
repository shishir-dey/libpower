pub struct IPark {
    alpha: f32,
    beta: f32,
    zero: f32,
    sin: f32,
    cos: f32,
    d: f32,
    q: f32,
    z: f32,
}

impl IPark {
    /// Creates a new instance of IPark with default values.
    pub fn new() -> IPark {
        IPark {
            alpha: 0.0,
            beta: 0.0,
            zero: 0.0,
            sin: 0.0,
            cos: 0.0,
            d: 0.0,
            q: 0.0,
            z: 0.0,
        }
    }

    /// Sets the d-q-z inputs for the inverse Park transform.
    pub fn set_input(&mut self, d: f32, q: f32, zero: f32) {
        self.d = d;
        self.q = q;
        self.z = zero;
    }

    /// Sets the rotation angle and computes its sine and cosine.
    pub fn set_angle(&mut self, theta: f32) {
        self.sin = libm::sinf(theta);
        self.cos = libm::cosf(theta);
    }

    /// Calculates the α-β components from the d-q inputs.
    pub fn calculate(&mut self) {
        self.alpha = self.d * self.cos - self.q * self.sin;
        self.beta = self.q * self.cos + self.d * self.sin;
        self.zero = self.z;
    }

    /// Retrieves the α-β-zero components.
    pub fn get_components(&self) -> (f32, f32, f32) {
        (self.alpha, self.beta, self.zero)
    }
}
