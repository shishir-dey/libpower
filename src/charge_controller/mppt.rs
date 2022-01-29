/*
* Version 1.0 | Shishir Dey | January 25th, 2022
* Description: Initial commit
*/

pub mod perturb_and_observe {
    enum VMPPAction {
        INCREMENT,
        DECREMENT,
    }
    #[allow(dead_code)]
    struct MPPT {
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
        #[allow(dead_code)]
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
        #[allow(dead_code)]
        pub fn get_mppt_v_out(&self) -> f32 {
            self.mppt_v_out
        }
        #[allow(dead_code)]
        pub fn calculate(&mut self, pv_i: f32, pv_v: f32) {
            if self.mppt_first {
                self.pv_v_prev = self.pv_v;
                self.pv_power_prev = self.pv_power;
                self.mppt_first = false;
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
}

pub mod incremental_conductance {
    enum VMPPAction {
        INCREMENT,
        DECREMENT,
    }
    #[allow(dead_code)]
    struct MPPT {
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

    #[allow(dead_code)]
    impl MPPT {
        #[allow(dead_code)]
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
        #[allow(dead_code)]
        pub fn get_mppt_v_out(&self) -> f32 {
            self.mppt_v_out
        }
        #[allow(dead_code)]
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

                let mut delta_pv_v_valid = false;
                let mut delta_pv_i_valid = false;
                if self.delta_pv_v > 0.0 {
                    if self.delta_pv_v > self.pv_v_high {
                        delta_pv_v_valid = true;
                    }
                } else {
                    if self.delta_pv_v < self.pv_v_low {
                        delta_pv_v_valid = true;
                    }
                }
                if self.delta_pv_i > 0.0 {
                    if self.delta_pv_i > self.pv_i_high {
                        delta_pv_i_valid = true;
                    }
                } else {
                    if self.delta_pv_i < self.pv_i_low {
                        delta_pv_i_valid = true;
                    }
                }
                if delta_pv_i_valid && delta_pv_v_valid {
                    if self.delta_pv_v > 0.0 {
                        if self.delta_pv_i == 0.0 {
                            if self.delta_pv_i == 0.0 {
                                self.pv_v_old = self.pv_v;
                                self.pv_i_old = self.pv_i;
                            } else {
                                if self.delta_pv_i > 0.0 {
                                    self.mppt_v_out_action = VMPPAction::DECREMENT;
                                } else {
                                    self.mppt_v_out_action = VMPPAction::INCREMENT;
                                }
                            }
                        } else {
                            self.conductance = -(self.pv_i / self.pv_v);
                            self.incremental_conductance = self.delta_pv_i / self.delta_pv_v;
                            if self.conductance == -self.incremental_conductance {
                                self.pv_v_old = self.pv_v;
                                self.pv_i_old = self.pv_i;
                            } else {
                                if self.incremental_conductance > -self.conductance {
                                    self.mppt_v_out_action = VMPPAction::DECREMENT;
                                } else {
                                    self.mppt_v_out_action = VMPPAction::INCREMENT;
                                }
                            }
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
            }
            self.pv_v_old = self.pv_v;
            self.pv_i_old = self.pv_i;
        }
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
