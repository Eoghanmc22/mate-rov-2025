#[derive(Clone)]
pub struct RunningAverage<const N: usize> {
    array: [f32; N],
    idx: usize,
    len: usize,
}

impl<const N: usize> RunningAverage<N> {
    pub const fn new() -> Self {
        Self {
            array: [0.0; N],
            idx: 0,
            len: 0,
        }
    }

    pub fn add_reading(&mut self, reading: f32) -> f32 {
        self.array[self.idx] = reading;

        self.idx += 1;
        self.idx %= N;

        if self.len < N {
            self.len += 1;
        }

        self.get_average()
    }

    pub fn get_average(&self) -> f32 {
        if self.len == 0 {
            return 0.0;
        }

        let sum = self.array[..self.len].iter().copied().sum::<f32>();
        sum / self.len as f32
    }
}

impl<const N: usize> Default for RunningAverage<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone)]
pub struct ExponentialMovingAverage {
    last: Option<f32>,
    alpha: f32,
}

impl ExponentialMovingAverage {
    pub const fn new(alpha: f32) -> Self {
        Self { last: None, alpha }
    }

    pub fn add_reading(&mut self, reading: f32) -> f32 {
        if let Some(last) = self.last {
            self.last = Some(reading * self.alpha + last * (1.0 - self.alpha));
        } else {
            self.last = Some(reading);
        }

        self.get_average()
    }

    pub fn get_average(&self) -> f32 {
        self.last.unwrap_or_default()
    }
}

impl Default for ExponentialMovingAverage {
    fn default() -> Self {
        Self {
            last: None,
            alpha: 0.3,
        }
    }
}
