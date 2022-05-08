/*
* Version 1.0 | Shishir Dey | January 24th, 2022
* Description: Initial commit. Currently supports only first order filter
*/

#[allow(dead_code)]
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
