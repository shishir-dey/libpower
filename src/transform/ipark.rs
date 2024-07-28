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
    pub fn new(alpha: f32, beta: f32) -> IPark {
        IPark {
            alpha: alpha,
            beta: beta,
            zero: 0.0,
            sin: 0.0,
            cos: 0.0,
            d: 0.0,
            q: 0.0,
            z: 0.0,
        }
    }
    pub fn calculate(&mut self) {
        self.alpha = self.d * self.cos - self.q * self.sin;
        self.beta = self.q * self.cos + self.d * self.sin;
    }
}
