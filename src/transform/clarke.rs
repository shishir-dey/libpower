pub struct Clarke {
    a: f32,
    b: f32,
    c: f32,
    alpha: f32,
    beta: f32,
    zero: f32,
}

impl Clarke {
    pub fn new(alpha: f32, beta: f32) -> Clarke {
        Clarke {
            a: 0.0,
            b: 0.0,
            c: 0.0,
            alpha: alpha,
            beta: beta,
            zero: 0.0,
        }
    }
    pub fn calculate(&mut self) {
        self.alpha = ((2.0 / 3.0) * self.a) - ((1.0 / 3.0) * (self.b - self.c));
        self.beta = (2.0 / 1.732) * (self.b - self.c);
        self.zero = 0.0;
    }
}
