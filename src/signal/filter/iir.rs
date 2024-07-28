pub struct IIRFilter {
    alpha: f32,
    out: f32,
    order: u8,
}

impl IIRFilter {
    #[allow(dead_code, unused_variables)]
    pub fn new(alpha: f32, order: u8) -> IIRFilter {
        IIRFilter {
            alpha: alpha,
            out: 0.0,
            order: 1,
        }
    }
    pub fn calculate(&mut self, input: f32) {
        self.out = self.alpha * input + (1.0 - self.alpha) * self.out;
    }
    pub fn get_out(&self) -> f32 {
        self.out
    }
}
