pub trait GenerateTrajectory {
    fn initialize(&mut self);
    fn step(&mut self) -> bool;
    fn time_required(&self) -> f32;
}

#[derive(Debug)]
pub enum ForwardPhase {
    Acceleration,
    ConstantVelocity,
    Deceleration,
}

pub struct ForwardTrajectory {
    pub x_target: f32,  // target position
    pub x_current: f32, // current position
    pub x_a: f32,       // acceleration distance
    pub x_d: f32,       // deceleration distance
    pub v_current: f32, // current velocity
    pub v_limit: f32,   // velocity_limit
    pub v_initial: f32, // initial velocity
    pub v_final: f32,   // final velocity
    pub a_accel: f32,   // acceleration
    pub a_decel: f32,   // deceleration

    pub phase: ForwardPhase,
}

impl ForwardTrajectory {
    pub fn new(x_t: f32, v_i: f32, v_l: f32, v_f: f32, a_a: f32, a_d: f32) -> Self {
        if (a_a <= 0.0) || (a_d <= 0.0) {
            panic!("a_a and a_d must be positive");
        }

        let mut v_l = v_l;
        let mut x_a = (v_l * v_l - v_i * v_i) / (2.0 * a_a);
        let mut x_d = (v_l * v_l - v_f * v_f) / (2.0 * a_d);
        if x_a + x_d > x_t {
            // Strict speed limits.
            v_l =
                ((2.0 * a_a * a_d * x_t + v_i * v_i * a_d - v_f * v_f * a_a) / (a_a + a_d)).sqrt();
            x_a = (v_l * v_l - v_i * v_i) / (2.0 * a_a);
            x_d = (v_l * v_l - v_f * v_f) / (2.0 * a_d);
        }

        if x_a + x_d > x_t {
            panic!("Cannot reach x_t")
        }

        ForwardTrajectory {
            x_target: x_t,
            x_current: 0.0,
            x_a,
            x_d,
            v_current: v_i,
            v_limit: v_l,
            v_initial: v_i,
            v_final: v_f,
            a_accel: a_a,
            a_decel: a_d,
            phase: ForwardPhase::Acceleration,
        }
    }
}

impl GenerateTrajectory for ForwardTrajectory {
    fn initialize(&mut self) {
        self.v_current = self.v_initial;
    }

    // return (x, y, theta, is_end)
    fn step(&mut self) -> bool {
        let mut is_end = false;
        self.x_current += self.v_current * 0.001;
        match self.phase {
            ForwardPhase::Acceleration => {
                self.v_current += self.a_accel * 0.001;
                if self.x_a <= self.x_current {
                    self.v_current = self.v_limit;
                    self.phase = ForwardPhase::ConstantVelocity;
                }
            }
            ForwardPhase::ConstantVelocity => {
                if self.x_target - self.x_d <= self.x_current {
                    self.phase = ForwardPhase::Deceleration;
                }
            }
            ForwardPhase::Deceleration => {
                self.v_current -= self.a_decel * 0.001;
                if self.x_target <= self.x_current {
                    self.v_current = self.v_final;
                    is_end = true;
                }
            }
        }
        is_end
    }

    fn time_required(&self) -> f32 {
        0.0
    }
}
