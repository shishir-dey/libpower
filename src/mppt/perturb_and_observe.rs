enum VMPPAction {
    INCREMENT,
    DECREMENT,
}
pub struct MPPT {
    pv_i: f32,
    pv_v: f32,
    pv_v_prev: f32,
    pv_power: f32,
    pv_power_prev: f32,
    delta_pv_power: f32,
    delta_p_min: f32,
    mppt_v_out_action: VMPPAction,
    mppt_v_out_max: f32,
    mppt_v_out_min: f32,
    step_size: f32,
    mppt_v_out: f32,
    mppt_enable: bool,
    mppt_first: bool,
}
impl MPPT {
    pub fn new() -> MPPT {
        MPPT {
            pv_i: 0.0,
            pv_v: 0.0,
            pv_v_prev: 0.0,
            pv_power: 0.0,
            pv_power_prev: 0.0,
            delta_pv_power: 0.0,
            delta_p_min: 0.0,
            mppt_v_out_action: VMPPAction::INCREMENT,
            mppt_v_out_max: 0.0,
            mppt_v_out_min: 0.0,
            step_size: 0.0,
            mppt_v_out: 0.0,
            mppt_enable: true,
            mppt_first: true,
        }
    }
    pub fn get_mppt_v_out(&self) -> f32 {
        self.mppt_v_out
    }

    pub fn get_pv_i(&self) -> f32 {
        self.pv_i
    }

    pub fn get_pv_v(&self) -> f32 {
        self.pv_v
    }
    pub fn get_pv_power(&self) -> f32 {
        self.pv_power
    }
    pub fn set_mppt_v_out_max(&mut self, value: f32) {
        self.mppt_v_out_max = value;
    }

    pub fn set_mppt_v_out_min(&mut self, value: f32) {
        self.mppt_v_out_min = value;
    }

    pub fn set_step_size(&mut self, value: f32) {
        self.step_size = value;
    }
    pub fn calculate(&mut self, pv_i: f32, pv_v: f32) {
        if self.mppt_first {
            self.pv_v_prev = self.pv_v;
            self.pv_power_prev = self.pv_power;
            self.mppt_first = false;
        } else {
            self.pv_i = pv_i;
            self.pv_v = pv_v;
            self.pv_power = self.pv_i * self.pv_v;

            // Calculate power change
            let delta_pv_power = self.pv_power - self.pv_power_prev;
            if delta_pv_power > self.delta_p_min {
                // Determine whether to INCREMENT or DECREMENT
                if self.pv_v > self.pv_v_prev {
                    self.mppt_v_out_action = VMPPAction::INCREMENT;
                } else {
                    self.mppt_v_out_action = VMPPAction::DECREMENT;
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
            }
            // Save the previous values
            self.pv_v_prev = self.pv_v;
            self.pv_power_prev = self.pv_power;
        }
    }
}
