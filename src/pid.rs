use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Default, Clone, Copy)]
pub struct PidParameter {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub i_limit: f32,
    pub dead_zone: f32,
}

pub struct Pid {
    p: f32,
    i: f32,
    d: f32,
    last_error: f32,
    integral: f32,
    i_limit: f32,
    dead_zone: f32,
}

impl Pid {
    pub fn new(parameter: PidParameter) -> Self {
        Pid {
            p: parameter.p,
            i: parameter.i,
            d: parameter.d,
            last_error: 0.0,
            integral: 0.0,
            i_limit: parameter.i_limit,
            dead_zone: parameter.dead_zone,
        }
    }

    pub fn update(&mut self, error: f32) -> f32 {
        let error = if error.abs() < self.dead_zone {
            0.0
        } else {
            error
        };

        // Proportional term
        let p = self.p * error;

        // Integral term
        self.integral += error;
        // Limit the integral term
        if self.integral > self.i_limit {
            self.integral = self.i_limit;
        }
        if self.integral < -self.i_limit {
            self.integral = -self.i_limit;
        }
        let i = self.i * self.integral;

        // Derivative term
        let d = self.d * (error - self.last_error);

        // Store last error
        self.last_error = error;

        // The output is the sum of the terms
        p + i + d
    }

    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.last_error = 0.0;
        self.integral = 0.0;
    }

    #[allow(dead_code)]
    pub fn set_p(&mut self, p: f32) {
        self.p = p;
    }

    #[allow(dead_code)]
    pub fn set_i(&mut self, i: f32) {
        self.i = i;
    }

    #[allow(dead_code)]
    pub fn set_d(&mut self, d: f32) {
        self.d = d;
    }

    #[allow(dead_code)]
    pub fn set_i_limit(&mut self, i_limit: f32) {
        self.i_limit = i_limit;
    }
}
