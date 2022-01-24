#[allow(dead_code)]
struct IIRFilterParams {
    alpha: f32,
    out: f32,
    order: u8,
}

impl IIRFilterParams {
    #[allow(dead_code, unused_variables)]
    pub fn new(alpha: f32, order: u8) -> IIRFilterParams {
        IIRFilterParams {
            alpha: alpha,
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

/* Placeholder for module's unit tests */
#[cfg(test)]
#[allow(unused_imports)]
mod tests {
    use super::*;

    #[test]
    fn test_will_always_fail() {
        assert!(false);
    }
}
