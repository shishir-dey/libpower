#![no_std]

#[derive(Clone, Copy)]
pub struct IClarke {
    a: f32,
    b: f32,
    c: f32,
    alpha: f32,
    beta: f32,
    zero: f32,
}

impl IClarke {
    pub fn new() -> Self {
        IClarke {
            a: 0.0,
            b: 0.0,
            c: 0.0,
            alpha: 0.0,
            beta: 0.0,
            zero: 0.0,
        }
    }

    pub fn set_input(&mut self, alpha: f32, beta: f32, zero: f32) {
        self.alpha = alpha;
        self.beta = beta;
        self.zero = zero;
    }

    pub fn calculate(&mut self) {
        const SQRT3: f32 = 1.732_050_8;

        // The inverse Clarke transform equations
        self.a = self.alpha + self.zero;
        self.b = (-0.5 * self.alpha) + (SQRT3 / 2.0 * self.beta) + self.zero;
        self.c = (-0.5 * self.alpha) - (SQRT3 / 2.0 * self.beta) + self.zero;
    }

    pub fn get_components(&self) -> (f32, f32, f32) {
        (self.a, self.b, self.c)
    }
}
