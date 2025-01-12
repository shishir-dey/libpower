#[derive(Debug, Clone)]
pub struct Coefficients {
    pub coeff_b2: f32,
    pub coeff_b1: f32,
    pub coeff_b0: f32,
    pub coeff_a2: f32,
    pub coeff_a1: f32,
    // Output saturation limits
    pub max: f32,
    pub i_min: f32,
    pub min: f32,
}

#[derive(Debug, Clone)]
pub struct Variables {
    pub out1: f32,
    pub out2: f32,
    // Internal values
    pub errn: f32,
    pub errn1: f32,
    pub errn2: f32,
    // Inputs
    pub ref_input: f32,
    pub fdbk: f32,
    // Output values
    pub out: f32,
}

impl Default for Coefficients {
    fn default() -> Self {
        Coefficients {
            coeff_b2: 0.0,
            coeff_b1: 0.0,
            coeff_b0: 0.0,
            coeff_a2: 0.0,
            coeff_a1: 0.0,
            max: f32::MAX,
            i_min: f32::MIN,
            min: f32::MIN,
        }
    }
}

impl Default for Variables {
    fn default() -> Self {
        Variables {
            out1: 0.0,
            out2: 0.0,
            errn: 0.0,
            errn1: 0.0,
            errn2: 0.0,
            ref_input: 0.0,
            fdbk: 0.0,
            out: 0.0,
        }
    }
}

impl Coefficients {
    pub fn new() -> Self {
        Default::default()
    }

    pub fn with_default_values() -> Self {
        Coefficients {
            coeff_b2: 0.3,
            coeff_b1: 0.2,
            coeff_b0: 0.1,
            coeff_a2: 0.2,
            coeff_a1: 0.1,
            max: f32::MAX,
            i_min: f32::MIN,
            min: f32::MIN,
        }
    }
}

impl Variables {
    pub fn new() -> Self {
        Default::default()
    }

    pub fn set_inputs(&mut self, ref_input: f32, fdbk: f32) {
        self.ref_input = ref_input;
        self.fdbk = fdbk;
    }
}

pub struct Controller2p2z {
    coeffs: Coefficients,
    vars: Variables,
}

impl Controller2p2z {
    pub fn new(coeffs: Coefficients) -> Self {
        Controller2p2z {
            coeffs,
            vars: Variables::default(),
        }
    }

    pub fn calculate(&mut self, ref_input: f32, fdbk: f32) -> f32 {
        // Update inputs
        self.vars.set_inputs(ref_input, fdbk);

        // Store previous errors
        self.vars.errn2 = self.vars.errn1;
        self.vars.errn1 = self.vars.errn;

        // Calculate new error
        self.vars.errn = self.vars.ref_input - self.vars.fdbk;

        // Store previous outputs
        self.vars.out2 = self.vars.out1;
        self.vars.out1 = self.vars.out;

        // Calculate new output
        let mut out = self.coeffs.coeff_b2 * self.vars.errn2
            + self.coeffs.coeff_b1 * self.vars.errn1
            + self.coeffs.coeff_b0 * self.vars.errn
            + self.coeffs.coeff_a2 * self.vars.out2
            + self.coeffs.coeff_a1 * self.vars.out1;

        // Apply saturation limits
        out = out.min(self.coeffs.max).max(self.coeffs.min);

        self.vars.out = out;
        out
    }

    pub fn get_output(&self) -> f32 {
        self.vars.out
    }

    pub fn get_error(&self) -> f32 {
        self.vars.errn
    }

    pub fn reset(&mut self) {
        self.vars = Variables::default();
    }
}
