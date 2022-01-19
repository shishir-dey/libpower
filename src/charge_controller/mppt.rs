enum VMPPAction {
    INCREMENT,
    DECREMENT,
}

#[allow(dead_code)]
struct MPPTParamsPerturbAndObserve {
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

impl MPPTParamsPerturbAndObserve {
    #[allow(dead_code)]
    pub fn new() -> MPPTParamsPerturbAndObserve {
        MPPTParamsPerturbAndObserve {
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

    #[allow(dead_code)]
    pub fn get_mppt_v_out(&self) -> f32 {
        self.mppt_v_out
    }

    #[allow(dead_code)]
    pub fn calculate(&mut self, pv_i: f32, pv_v: f32) {
        if self.mppt_first {
            self.pv_v_prev = self.pv_v;
            self.pv_power_prev = self.pv_power;
        } else {
            self.pv_i = pv_i;
            self.pv_v = pv_v;
            self.pv_power = self.pv_i * self.pv_v;
            if self.pv_power > self.pv_power_prev {
                self.delta_pv_power = self.pv_power - self.pv_power_prev;
            } else {
                self.delta_pv_power = self.pv_power_prev - self.pv_power;
            }
            if self.delta_pv_power > self.delta_p_min {
                if self.pv_power > self.pv_power_prev {
                    if self.pv_v > self.pv_v_prev {
                        self.mppt_v_out_action = VMPPAction::INCREMENT;
                    } else {
                        self.mppt_v_out_action = VMPPAction::DECREMENT;
                    }
                } else {
                    if self.pv_v > self.pv_v_prev {
                        self.mppt_v_out_action = VMPPAction::DECREMENT;
                    } else {
                        self.mppt_v_out_action = VMPPAction::INCREMENT;
                    }
                }

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
            self.pv_v_prev = self.pv_v;
            self.pv_power_prev = self.pv_power;
        }
    }
}
