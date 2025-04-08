use bevy::app::App;

pub mod system;
pub mod units;
pub mod utils;

pub fn register_types(app: &mut App) {
    system::register_types(app);
    units::register_types(app);
    utils::register_types(app);
}
