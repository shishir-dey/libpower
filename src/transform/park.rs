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
    pub fn new(alpha: f32, beta: f32) -> Park {
        Park {
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
        self.d = self.alpha * self.cos + self.beta * self.sin;
        self.q = self.beta * self.cos - self.alpha * self.sin;
        self.z = 0.0;
    }
}
