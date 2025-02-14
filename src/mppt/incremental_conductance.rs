enum VMPPAction {
    INCREMENT,
    DECREMENT,
}
pub struct MPPT {
    pv_i: f32,
    pv_v: f32,
    pv_i_high: f32,
    pv_i_low: f32,
    pv_v_high: f32,
    pv_v_low: f32,
    step_size: f32,
    mppt_v_out: f32,
    mppt_v_out_action: VMPPAction,
    mppt_v_out_max: f32,
    mppt_v_out_min: f32,
    conductance: f32,
    incremental_conductance: f32,
    delta_pv_v: f32,
    delta_pv_i: f32,
    pv_v_old: f32,
    pv_i_old: f32,
    mppt_enable: bool,
    mppt_first: bool,
}
impl MPPT {
    pub fn new() -> MPPT {
        MPPT {
            pv_i: 0.0,
            pv_v: 0.0,
            pv_i_high: 0.0,
            pv_i_low: 0.0,
            pv_v_high: 0.0,
            pv_v_low: 0.0,
            step_size: 0.0,
            mppt_v_out: 0.0,
            mppt_v_out_action: VMPPAction::INCREMENT,
            mppt_v_out_max: 0.0,
            mppt_v_out_min: 0.0,
            conductance: 0.0,
            incremental_conductance: 0.0,
            delta_pv_v: 0.0,
            delta_pv_i: 0.0,
            pv_v_old: 0.0,
            pv_i_old: 0.0,
            mppt_enable: true,
            mppt_first: true,
        }
    }
    pub fn get_pv_i(&self) -> f32 {
        self.pv_i
    }

    pub fn get_pv_v(&self) -> f32 {
        self.pv_v
    }

    pub fn get_pv_i_high(&self) -> f32 {
        self.pv_i_high
    }

    pub fn get_pv_v_high(&self) -> f32 {
        self.pv_v_high
    }

    pub fn get_mppt_v_out(&self) -> f32 {
        self.mppt_v_out
    }
    pub fn set_mppt_v_out_max(&mut self, mppt_v_out_max: f32) {
        self.mppt_v_out_max = mppt_v_out_max;
    }

    pub fn set_mppt_v_out_min(&mut self, mppt_v_out_min: f32) {
        self.mppt_v_out_min = mppt_v_out_min;
    }

    pub fn set_step_size(&mut self, step_size: f32) {
        self.step_size = step_size;
    }
    pub fn calculate(&mut self, pv_i: f32, pv_v: f32) {
        if self.mppt_first {
            self.pv_v_old = self.pv_v;
            self.pv_i_old = self.pv_i;
            self.mppt_first = false;
        } else {
            self.pv_i = pv_i;
            self.pv_v = pv_v;
            self.delta_pv_i = self.pv_i - self.pv_i_old;
            self.delta_pv_v = self.pv_v - self.pv_v_old;

            // Determine if the conductance is positive or negative
            if self.delta_pv_v > 0.0 {
                if self.delta_pv_i > 0.0 {
                    self.mppt_v_out_action = VMPPAction::INCREMENT;
                } else {
                    self.mppt_v_out_action = VMPPAction::DECREMENT;
                }
            } else {
                if self.delta_pv_i < 0.0 {
                    self.mppt_v_out_action = VMPPAction::INCREMENT;
                } else {
                    self.mppt_v_out_action = VMPPAction::DECREMENT;
                }
            }

            // Adjust voltage output based on action
            match self.mppt_v_out_action {
                VMPPAction::INCREMENT => {
                    if self.mppt_v_out + self.step_size > self.mppt_v_out_max {
                        self.mppt_v_out = self.mppt_v_out_max;
                    } else {
                        self.mppt_v_out += self.step_size;
                    }
                }
                VMPPAction::DECREMENT => {
                    if self.mppt_v_out - self.step_size < self.mppt_v_out_min {
                        self.mppt_v_out = self.mppt_v_out_min;
                    } else {
                        self.mppt_v_out -= self.step_size;
                    }
                }
            }
            // Save the previous values
            self.pv_v_old = self.pv_v;
            self.pv_i_old = self.pv_i;
        }
    }
}
