use std::time::Duration;

pub mod device;
pub mod units;

/// Represents a locally available sensor or actuator
///
/// This trait's init method must be called exactly once before any other methods are used
///
/// Data is refreshed and or sent to the device when the poll method is called
/// The results of all other methods should be cached
///
pub trait Hardware /*: TryFrom<Config> */ {
    // FIXME: I dont like this method
    fn init(&mut self) -> anyhow::Result<()>;
    fn poll(&mut self) -> anyhow::Result<()>;
    /// Returns the shortest interval after which the device is guaranteed to have new data
    fn fastest_polling_interval(&self) -> anyhow::Result<Option<Duration>>;
    /// Returns the default/suggested polling interval
    fn suggested_polling_interval(&self) -> anyhow::Result<Duration>;
}
