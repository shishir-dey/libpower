#[derive(Debug, Clone)]
pub struct ControllerPI {
    // Inputs
    ref_input: f32, // reference set-point
    fdbk: f32,      // feedback

    // Outputs
    out: f32, // controller output

    // Parameters
    kp: f32,    // proportional gain
    ki: f32,    // integral gain
    u_max: f32, // upper saturation limit
    u_min: f32, // lower saturation limit

    // Internal state
    up: f32, // proportional term
    ui: f32, // integral term
    v1: f32, // pre-saturated controller output
    i1: f32, // integrator storage: ui(k-1)
    w1: f32, // saturation record: [u(k-1) - v(k-1)]
}

impl Default for ControllerPI {
    fn default() -> Self {
        ControllerPI {
            ref_input: 0.0,
            fdbk: 0.0,
            out: 0.0,
            kp: 0.2,
            ki: 0.1,
            u_max: f32::MAX,
            u_min: f32::MIN,
            up: 0.0,
            ui: 0.0,
            v1: 0.0,
            i1: 0.0,
            w1: 0.0,
        }
    }
}

impl ControllerPI {
    pub fn new() -> Self {
        Default::default()
    }

    pub fn get_kp(&self) -> f32 {
        self.kp
    }

    pub fn get_ki(&self) -> f32 {
        self.ki
    }

    pub fn with_gains(kp: f32, ki: f32) -> Self {
        let mut controller = Self::default();
        controller.kp = kp;
        controller.ki = ki;
        controller
    }

    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.kp = kp;
        self.ki = ki;
    }

    pub fn set_limits(&mut self, u_min: f32, u_max: f32) {
        self.u_min = u_min;
        self.u_max = u_max;
    }

    pub fn calculate(&mut self, ref_input: f32, fdbk: f32) -> f32 {
        self.ref_input = ref_input;
        self.fdbk = fdbk;

        // Calculate error
        let error = self.ref_input - self.fdbk;

        // Compute proportional term
        self.up = self.kp * error;

        // Compute integral term with anti-windup
        self.ui = self.i1 + (self.ki * error) + (self.w1);

        // Compute controller output
        self.v1 = self.up + self.ui;

        // Apply saturation limits
        self.out = self.v1.min(self.u_max).max(self.u_min);

        // Compute saturation record for anti-windup
        self.w1 = self.out - self.v1;

        // Store integral term for next iteration
        self.i1 = self.ui;

        self.out
    }

    pub fn reset(&mut self) {
        self.up = 0.0;
        self.ui = 0.0;
        self.v1 = 0.0;
        self.i1 = 0.0;
        self.w1 = 0.0;
        self.out = 0.0;
    }

    // Getters
    pub fn get_output(&self) -> f32 {
        self.out
    }

    pub fn get_proportional_term(&self) -> f32 {
        self.up
    }

    pub fn get_integral_term(&self) -> f32 {
        self.ui
    }
}
