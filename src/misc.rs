/********** Correction by table **********/
pub fn correct_value<T: PartialOrd + Copy + Into<f32>>(table: &[(T, f32)], raw_value: T) -> f32 {
    if table.is_empty() {
        return 0.0;
    }

    let raw_value_f32: f32 = raw_value.into();
    let below_limit_value = table[0].1;
    let above_limit_value = table[table.len() - 1].1;

    if raw_value_f32 < table[0].0.into() {
        return below_limit_value;
    }

    if raw_value_f32 > table[table.len() - 1].0.into() {
        return above_limit_value;
    }

    // Assuming the correction table is sorted, find the appropriate correction value
    for i in 0..table.len() {
        if raw_value_f32 == table[i].0.into() {
            return table[i].1;
        } else if i < table.len() - 1
            && raw_value_f32 > table[i].0.into()
            && raw_value_f32 < table[i + 1].0.into()
        {
            // 線形補間を使用して値を補正
            let fraction =
                (raw_value_f32 - table[i].0.into()) / (table[i + 1].0.into() - table[i].0.into());
            return table[i].1 + fraction * (table[i + 1].1 - table[i].1);
        }
    }

    // This line should not actually be reached, but is added to avoid compiler errors
    above_limit_value
}

/* Usage */
/*
fn main() {
    let table: [(u16, f32); 3] = [(0, 0.0), (100, 1.0), (240, 2.0)];
    let raw_value: u16 = 89;
    let below_limit_value: f32 = -1.0;
    let above_limit_value: f32 = 3.0;

    let corrected = correct_value(&table, raw_value, below_limit_value, above_limit_value);
    println!("Corrected value: {}", corrected);
}
*/

/********** FIR filter **********/

pub struct FIR<T> {
    coefficients: Vec<T>,
    buffer: Vec<T>,
}

impl<T> FIR<T>
where
    T: Default + Copy + std::ops::Add<Output = T> + std::ops::Mul<Output = T>,
{
    pub fn new(coefficients: Vec<T>) -> Self {
        let buffer = vec![T::default(); coefficients.len()];
        FIR {
            coefficients,
            buffer,
        }
    }

    pub fn filter(&mut self, input: T) -> T {
        // Adds the new input value to the beginning of the buffer and deletes the old value
        self.buffer.remove(0);
        self.buffer.push(input);

        // FIR filter
        let mut output = T::default();
        for i in 0..self.coefficients.len() {
            output = output + self.buffer[i] * self.coefficients[i];
        }
        output
    }

    pub fn reset(&mut self) {
        for item in self.buffer.iter_mut() {
            *item = T::default();
        }
    }
}

/* Usage */
/*
fn main() {
    // e.g. FIR filter with 3 coefficients
    let coefficients = vec![0.2, 0.5, 0.3];
    let mut fir = FIR::new(coefficients);

    let input_data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
    for &sample in &input_data {
        let output = fir.filter(sample);
        println!("Input: {}, Output: {}", sample, output);
    }
}
*/

#[allow(dead_code)]
pub fn rad(deg: f32) -> f32 {
    deg * std::f32::consts::PI / 180.0
}

#[allow(dead_code)]
pub fn deg(rad: f32) -> f32 {
    rad * 180.0 / std::f32::consts::PI
}
pub struct MovingAverage {
    window_size: usize,
    values: Vec<f32>,
}

impl MovingAverage {
    pub fn new(window_size: usize) -> Self {
        MovingAverage {
            window_size,
            values: Vec::with_capacity(window_size),
        }
    }

    pub fn update(&mut self, value: f32) -> f32 {
        if self.values.len() == self.window_size {
            self.values.remove(0);
        }
        self.values.push(value);
        let mut sum = 0.0;
        for v in &self.values {
            sum += v;
        }
        sum / self.values.len() as f32
    }

    pub fn reset(&mut self) {
        self.values.clear();
    }

    pub fn set_window_size(&mut self, window_size: usize) {
        self.window_size = window_size;
        self.reset();
    }
}

pub struct MovingAverageInt {
    window_size: usize,
    values: Vec<i32>,
    sum: i32,
}

impl MovingAverageInt {
    pub fn new(window_size: usize) -> Self {
        MovingAverageInt {
            window_size,
            values: Vec::with_capacity(window_size),
            sum: 0,
        }
    }

    pub fn update(&mut self, value: i32) -> i32 {
        if self.values.len() == self.window_size {
            self.sum -= self.values.remove(0);
        }
        self.values.push(value);
        self.sum += value;
        self.sum / self.values.len() as i32
    }

    pub fn reset(&mut self) {
        self.values.clear();
        self.sum = 0;
    }

    pub fn set_window_size(&mut self, window_size: usize) {
        self.window_size = window_size;
        self.reset();
    }
}
