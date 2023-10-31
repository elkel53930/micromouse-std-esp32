pub struct Pid {
    p: f32,
    i: f32,
    d: f32,
    last_error: f32,
    integral: f32,
    i_limit: f32,
}

impl Pid {
    pub fn new(p: f32, i: f32, d: f32, i_limit: f32) -> Self {
        Pid {
            p,
            i,
            d,
            last_error: 0.0,
            integral: 0.0,
            i_limit: i_limit,
        }
    }

    pub fn update(&mut self, error: f32) -> f32 {
        let p = self.p * error;
        self.integral += error;
        if self.integral > self.i_limit {
            self.integral = self.i_limit;
        }
        if self.integral < -self.i_limit {
            self.integral = -self.i_limit;
        }

        let i = self.i * self.integral;
        let d = self.d * (error - self.last_error);
        self.last_error = error;
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
