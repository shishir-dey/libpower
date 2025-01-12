#[cfg(test)]
mod tests {
    use libpower::control::cntl_pid::ControllerPID;

    #[test]
    fn test_pid_proportional_only() {
        let mut pid = ControllerPID::new(2.0, 0.0, 0.0); // Only proportional control
        let setpoint = 10.0;
        let current_position = 8.0;
        let current_time = 1.0;

        let output = pid.update(setpoint, current_position, current_time);

        // P = kp * error
        let error = setpoint - current_position;
        let expected_output = 2.0 * error;
        assert!((output - expected_output).abs() < 1e-6);
    }

    #[test]
    fn test_pid_derivative_only() {
        let mut pid = ControllerPID::new(0.0, 0.0, 1.0); // Only derivative control
        let setpoint = 10.0;
        let current_position1 = 8.0;
        let current_position2 = 9.0;
        let current_time1 = 1.0;
        let current_time2 = 2.0;

        let _ = pid.update(setpoint, current_position1, current_time1); // First update
        let output = pid.update(setpoint, current_position2, current_time2); // Second update

        // D = kd * (delta_position / delta_time)
        let delta_position = current_position2 - current_position1;
        let delta_time = current_time2 - current_time1;
        let expected_output = 1.0 * (delta_position / delta_time);
        assert!((output - expected_output).abs() < 1e-6);
    }

    #[test]
    fn test_pid_zero_error() {
        let mut pid = ControllerPID::new(1.0, 0.1, 0.01);
        let setpoint = 10.0;
        let current_position = 10.0;
        let current_time = 1.0;

        let output = pid.update(setpoint, current_position, current_time);

        assert_eq!(output, 0.0); // No error, output should be zero
    }
}
