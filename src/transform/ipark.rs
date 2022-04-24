/*
* Version 1.0 | Shishir Dey | April 24th, 2022
* Description: Initial commit
*/

#[allow(dead_code)]
struct IPark {
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
    #[allow(dead_code)]
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
    #[allow(dead_code)]
    pub fn calculate(&mut self) {
        self.alpha = self.d * self.cos - self.q * self.sin;
        self.beta = self.q * self.cos + self.d * self.sin;
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
