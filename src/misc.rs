/********** Correction by table **********/
pub fn correct_value<T: PartialOrd + Copy + Into<f32>>(
    table: &[(T, f32)],
    raw_value: T,
    below_limit_value: f32,
    above_limit_value: f32,
) -> f32 {
    let raw_value_f32: f32 = raw_value.into();

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
