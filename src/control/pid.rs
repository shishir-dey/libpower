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
    pub fn new(kp: f32, ki: f32, kd: f32) -> PID {
        PID {
            kp,
            ki,
            kd,
            last_position: 0.0,
            previous_time: 0.0,
            current_time: 0.0,
            first_pass: true,
            cumulative_error: 0.0,
        }
    }

    pub fn update(&mut self, setpoint: f32, current_position: f32, current_time: f32) -> f32 {
        // Calculate delta_time
        let delta_time = current_time - self.previous_time;
        if delta_time <= 0.0 {
            return 0.0; // Skip updates for invalid or zero time intervals
        }

        // Calculate error
        let error = setpoint - current_position;

        // Update cumulative error (integral term)
        self.cumulative_error += error * delta_time;

        // Calculate delta_position (for derivative term)
        let delta_position = current_position - self.last_position;

        // Update internal state
        self.last_position = current_position;
        self.previous_time = current_time;

        // Compute PID terms
        let p_term = self.kp * error;
        let i_term = self.ki * self.cumulative_error;
        let d_term = if self.first_pass {
            0.0 // Skip derivative term on the first pass
        } else {
            self.kd * delta_position / delta_time
        };

        // Update first pass flag
        self.first_pass = false;

        // Compute and return output
        p_term + i_term + d_term
    }

    pub fn cumulative_error(&self) -> f32 {
        self.cumulative_error
    }

    pub fn last_position(&self) -> f32 {
        self.last_position
    }

    pub fn reset(&mut self) {
        self.last_position = 0.0;
        self.previous_time = 0.0;
        self.current_time = 0.0;
        self.first_pass = true;
        self.cumulative_error = 0.0;
    }
}
