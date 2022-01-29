/*
* Version 1.0 | Shishir Dey | January 24th, 2022
* Description: Initial commit
*/

struct KalmanFilter {
    x_est: f32,      /* Output estimate */
    x_est_last: f32, /* Last output estimate */
    x_temp_est: f32, /* Temporary output estimate */
    p: f32,          /* Prediction */
    p_temp: f32,     /* Temporary prediction */
    p_last: f32,     /* Last prediction */
    k: f32,          /* Kalman gain */
    q: f32,          /* Process noise covariance */
    r: f32,          /* Measurement noise covariance */
}

impl KalmanFilter {
    #[allow(dead_code)]
    pub fn new(q: f32, r: f32) -> KalmanFilter {
        KalmanFilter {
            x_est: 0.0,
            x_est_last: 0.0,
            x_temp_est: 0.0,
            p: 0.0,
            p_temp: 0.0,
            p_last: 0.0,
            k: 0.0,
            q: q,
            r: r,
        }
    }
    #[allow(dead_code)]
    pub fn get_output_estimate(&self) -> f32 {
        self.x_est
    }
    #[allow(dead_code)]
    pub fn update(&mut self, z: f32) {
        self.x_temp_est = self.x_est_last;
        self.p_temp = self.p_last + self.q;
        self.k = self.p_temp / (self.p_temp + self.r);
        self.x_est = self.x_temp_est + self.k * (z - self.x_temp_est);
        self.p = (1.0 - self.k) * self.p_temp;
        self.p_last = self.p;
        self.x_est_last = self.x_est;
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
