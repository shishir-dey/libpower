/*
* Version 1.0 | Shishir Dey | January 15th, 2022
* Description: Initial commit
*/

#[allow(dead_code)]
pub struct PID {
    kp: f32,
    ki: f32,
    kd: f32,
    last_position: f32,
    previous_time: f32,
    current_time: f32,
    first_pass: bool,
    cumulative_error: f32,
}

impl PID {
    #[allow(dead_code)]
    pub fn new(kp: f32, ki: f32, kd: f32) -> PID {
        PID {
            kp: kp,
            ki: ki,
            kd: kd,
            last_position: 0.0,
            previous_time: 0.0,
            current_time: 0.0,
            first_pass: true,
            cumulative_error: 0.0,
        }
    }
    #[allow(dead_code)]
    pub fn update(&mut self, setpoint: f32, current_position: f32, current_time: f32) -> f32 {
        self.current_time = current_time;
        let delta_time = self.current_time - self.previous_time;
        let error = setpoint - current_position;
        self.cumulative_error += error * delta_time;
        let delta_position = current_position - self.last_position;
        self.last_position = current_position;
        self.previous_time = self.current_time;
        let p_term = self.kp * error;
        let i_term = self.ki * self.cumulative_error;
        let d_term = self.kd * delta_position / delta_time;
        let output = p_term + i_term + d_term;
        if self.first_pass {
            self.first_pass = false;
            output
        } else {
            output
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
