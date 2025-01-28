pub trait Motor {
    /// Input from -1.0 to 1.0 where 0.0 is stopped
    ///
    /// DC motor: input -> power/duty cycle
    /// BLDC motor: input -> ?
    /// Servo: input -> angle
    /// Continuous servo: input -> speed
    fn set_input(&self, input: f32) -> anyhow::Result<()>;
}
