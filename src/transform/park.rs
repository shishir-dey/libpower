#[derive(Clone, Copy)]
pub struct Park {
    alpha: f32,
    beta: f32,
    zero: f32,
    sin: f32,
    cos: f32,
    d: f32,
    q: f32,
    z: f32,
}

impl Park {
    pub fn new() -> Self {
        Park {
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

    pub fn set_input(&mut self, alpha: f32, beta: f32, zero: f32) {
        self.alpha = alpha;
        self.beta = beta;
        self.zero = zero;
    }

    pub fn set_angle(&mut self, theta: f32) {
        // Expected theta in radians
        self.sin = libm::sinf(theta);
        self.cos = libm::cosf(theta);
    }

    pub fn calculate(&mut self) {
        self.d = self.alpha * self.cos + self.beta * self.sin;
        self.q = self.beta * self.cos - self.alpha * self.sin;
        self.z = self.zero;
    }

    pub fn get_components(&self) -> (f32, f32, f32) {
        (self.d, self.q, self.z)
    }
}
