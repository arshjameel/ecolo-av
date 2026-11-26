//! The hardware panel as a page: what `hw_panel` (main.rs) shows, and what the map
//! window (map-app) shows as its HARDWARE page. One implementation, two hosts.
//!
//! | Part | Does |
//! |---|---|
//! | [`page::HardwarePage`] | the panel between its source and a window: news in, the keys and the mouse, the buttons' rules, one frame drawn, the command out |
//! | [`page::Panel`] | the same without a window: news, where it may drive, the buttons (the Ravensim check drives it) |
//! | [`source`] | where the news comes from: the ROS node `hw_panel` ([`ros`]), or the [`demo`] |
//! | [`open_source`] | starts either, with the ESP32's lines beside it when asked |
//! | [`ros_args`] | the ROS arguments the node `hw_panel` gets inside another program |
//! | [`ui`] | the drawing, raylib |
//!
//! The rules (where the buttons may drive, the heartbeat, the e-stop) are panel-core's,
//! the same in both hosts.

pub mod cal_script;
pub mod demo;
mod esp32;
pub mod page;
mod robot;
#[cfg(feature = "ros")]
pub mod ros;
pub mod ros_args;
pub mod source;
pub mod ui;

use std::time::Instant;

use anyhow::{Context as _, Result};
use panel_core::config::PanelConfig;

pub use page::{HardwarePage, Host, Input, Panel, Step};

use crate::{
    demo::Demo,
    esp32::Esp32Reader,
    source::{DataSource, WithEsp32},
};

/// Where the panel's news comes from.
#[derive(Debug, Clone, Default)]
pub struct SourceOptions {
    /// The pretend car: no ROS.
    pub demo: bool,
    /// The demo's stiff spot in the steering, counts.
    pub demo_bind: Option<u16>,
    /// `--esp32 ws://...`: wins over `esp32_url` in the config or the parameters.
    pub esp32: Option<String>,
}

/// The demo, or the ROS node `hw_panel` (which may rename topics from its parameters),
/// with the ESP32's status lines beside it when `--esp32` or `esp32_url` asks.
/// `args`: the command line the node reads its `--ros-args` from. `who`: the program,
/// for its messages.
pub fn open_source(
    args: Vec<String>,
    mut config: PanelConfig,
    opts: &SourceOptions,
    who: &str,
) -> Result<(Box<dyn DataSource>, PanelConfig)> {
    if let Some(url) = &opts.esp32 {
        config.esp32_url = url.clone();
    }
    let (source, mut config): (Box<dyn DataSource>, PanelConfig) = if opts.demo {
        // With a real ESP32's lines, the demo pretends none of its own.
        let lines = config.esp32_url.trim().is_empty();
        let demo = Demo::new(Instant::now(), opts.demo_bind, lines);
        (Box::new(demo), config)
    } else {
        start_node(args, config, who)?
    };
    // --esp32 wins over esp32_url from a parameters file.
    if let Some(url) = &opts.esp32 {
        config.esp32_url = url.clone();
    }
    let source: Box<dyn DataSource> = match config.esp32().context("esp32_url")? {
        Some(url) => {
            println!("{who}: reading the ESP32's status lines from {url} (read only)");
            Box::new(WithEsp32 {
                inner: source,
                esp32: Esp32Reader::start(url)?,
            })
        }
        None => source,
    };
    Ok((source, config))
}

#[cfg(feature = "ros")]
fn start_node(
    args: Vec<String>,
    config: PanelConfig,
    _who: &str,
) -> Result<(Box<dyn DataSource>, PanelConfig)> {
    let (source, config) = ros::RosSource::start(args, config)?;
    Ok((Box::new(source), config))
}

#[cfg(not(feature = "ros"))]
fn start_node(
    _args: Vec<String>,
    _config: PanelConfig,
    who: &str,
) -> Result<(Box<dyn DataSource>, PanelConfig)> {
    anyhow::bail!("this {who} was built without ROS (--no-default-features): run it with --demo")
}
