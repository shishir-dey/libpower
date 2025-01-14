#[derive(Clone, Copy)]
pub struct Clarke {
    a: f32,
    b: f32,
    c: f32,
    alpha: f32,
    beta: f32,
    zero: f32,
}

impl Clarke {
    pub fn new() -> Self {
        Clarke {
            a: 0.0,
            b: 0.0,
            c: 0.0,
            alpha: 0.0,
            beta: 0.0,
            zero: 0.0,
        }
    }

    pub fn set_input(&mut self, a: f32, b: f32, c: f32) {
        self.a = a;
        self.b = b;
        self.c = c;
    }

    pub fn calculate(&mut self) {
        const SQRT3: f32 = 1.732_050_8; // √3 manually defined

        self.zero = (self.a + self.b + self.c) / 3.0;
        self.alpha = (2.0 * self.a - self.b - self.c) / 3.0;
        self.beta = (2.0 / SQRT3) * (self.b - self.c);
    }

    pub fn get_components(&self) -> (f32, f32, f32) {
        (self.alpha, self.beta, self.zero)
    }
}
