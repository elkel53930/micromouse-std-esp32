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
    Stop,
}

#[derive(Debug)]
pub struct ForwardTrajectory {
    pub x_initial: f32, // initial position
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

    pub stop_count: u16,

    pub phase: ForwardPhase,
}

impl ForwardTrajectory {
    pub fn new(
        x_initial: f32,
        x_distance: f32,
        v_initial: f32,
        v_limit: f32,
        v_final: f32,
        a_acc: f32,
        a_dec: f32,
    ) -> Self {
        if (a_acc <= 0.0) || (a_dec <= 0.0) {
            panic!("a_a and a_d must be positive");
        }

        let mut v_limit = v_limit;
        let mut x_acc = (v_limit * v_limit - v_initial * v_initial) / (2.0 * a_acc);
        let mut x_dec = (v_limit * v_limit - v_final * v_final) / (2.0 * a_dec);
        if x_acc + x_dec > x_distance {
            // Strict speed limits.
            v_limit = ((2.0 * a_acc * a_dec * x_distance + v_initial * v_initial * a_dec
                - v_final * v_final * a_acc)
                / (a_acc + a_dec))
                .sqrt();
            x_acc = (v_limit * v_limit - v_initial * v_initial) / (2.0 * a_acc);
            x_dec = (v_limit * v_limit - v_final * v_final) / (2.0 * a_dec);
        }

        if x_acc + x_dec > x_distance {
            panic!("Cannot reach x_t")
        }

        let x_target = x_distance + x_initial;

        ForwardTrajectory {
            x_initial,
            x_target: x_target,
            x_current: x_initial,
            x_a: x_acc + x_initial,
            x_d: x_target - x_dec,
            v_current: v_initial,
            v_limit,
            v_initial,
            v_final,
            a_accel: a_acc,
            a_decel: a_dec,
            phase: ForwardPhase::Acceleration,
            stop_count: 0,
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
                if self.x_d <= self.x_current {
                    self.phase = ForwardPhase::Deceleration;
                }
            }
            ForwardPhase::Deceleration => {
                self.v_current -= self.a_decel * 0.001;
                if self.v_current < 0.0 {
                    self.v_current = 0.05;
                }
                if self.x_target <= self.x_current {
                    self.v_current = self.v_final;
                    if self.v_final < 0.001 {
                        self.phase = ForwardPhase::Stop;
                        self.stop_count = 0;
                    } else {
                        is_end = true;
                    }
                }
            }
            ForwardPhase::Stop => {
                self.v_current = self.v_final;
                self.x_current = self.x_target;
                if self.stop_count > 50 {
                    is_end = true
                } else {
                    self.stop_count += 1;
                }
            }
        }
        is_end
    }

    fn time_required(&self) -> f32 {
        0.0
    }
}
