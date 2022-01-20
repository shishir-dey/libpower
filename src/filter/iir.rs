#[allow(dead_code)]
struct IIRFilterParams {
    alpha: f32,
    out: f32,
    order: u8,
}

impl IIRFilterParams {
    #[allow(dead_code, unused_variables)]
    pub fn new(order: u8) -> IIRFilterParams {
        IIRFilterParams {
            alpha: 0.0,
            out: 0.0,
            order: 1,
        }
    }
    #[allow(dead_code)]
    pub fn calculate(&mut self, input: f32) {
        self.out = self.alpha * input + (1.0 - self.alpha) * self.out;
    }
    #[allow(dead_code)]
    pub fn get_out(&self) -> f32 {
        self.out
    }
}
