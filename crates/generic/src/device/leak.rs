pub trait LeakSensor {
    /// Returns true if a leak is detected
    fn read(&self) -> anyhow::Result<bool>;
}
