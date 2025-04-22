use bevy::app::App;

pub mod system;
pub mod units;

pub fn register_types(app: &mut App) {
    system::register_types(app);
    units::register_types(app);
}
