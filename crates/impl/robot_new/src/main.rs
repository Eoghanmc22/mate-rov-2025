use std::fs;

use anyhow::Context;

use crate::config::Config;

pub mod config;

fn main() -> anyhow::Result<()> {
    let config = fs::read_to_string("robot.toml").context("Read config")?;
    let config: Config = toml::from_str(&config).context("Parse config")?;

    println!("Config: {config:#?}");

    Ok(())
}
