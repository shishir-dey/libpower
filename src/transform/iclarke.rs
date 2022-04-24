/*
* Version 1.0 | Shishir Dey | April 24th, 2022
* Description: Initial commit
*/

#[allow(dead_code)]
struct IClarke {
    a: f32,
    b: f32,
    c: f32,
    alpha: f32,
    beta: f32,
    zero: f32,
}

impl IClarke {
    #[allow(dead_code)]
    pub fn new(alpha: f32, beta: f32) -> IClarke {
        IClarke {
            a: 0.0,
            b: 0.0,
            c: 0.0,
            alpha: alpha,
            beta: beta,
            zero: 0.0,
        }
    }
    #[allow(dead_code)]
    pub fn calculate(&mut self) {
        self.a = self.alpha;
        self.b = 0.5 * (-self.alpha + (1.732 * self.beta));
        self.c = 0.5 * (-self.alpha - (1.732 * self.beta));
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
