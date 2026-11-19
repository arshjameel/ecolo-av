//! AMCL for a car: where it is on a 2D map, from a laser scan and odometry.
//!
//! A cloud of guesses (particles) moves with the odometry, plus noise; each is weighed by
//! how well the scan fits the map from there; the good ones are copied, the bad ones
//! dropped. A port of nav2_amcl's behaviour (Fox, Burgard, Dellaert and Thrun 1999;
//! Thrun, Burgard and Fox, *Probabilistic Robotics*, 2005), rewritten, not transliterated.
//! No ROS here: `amcl-node` feeds it messages.
//!
//! | Module | Holds |
//! |---|---|
//! | [`pose`] | [`Pose2`] (x, y, yaw), and the units [`Metres`] and [`Radians`] |
//! | [`rng`] | a small seeded random source ([`Rng`]) |
//! | [`map`] | the [`OccupancyMap`], its [`DistanceField`] and its [`FreeSpace`] |
//! | [`motion`] | the odometry motion model ([`DifferentialMotion`], nav2's alpha1-4) |
//! | [`scan`] | the [`Scan`], the [`ScanModel`] trait, and its [`LikelihoodField`] |
//! | [`filter`] | the [`ParticleFilter`]: KLD count, low-variance resampling, recovery, the [`Estimate`] |
//! | [`odom`] | the [`OdomBuffer`]: the odometry at a scan's stamp |
//! | [`amcl`] | all of it per scan: [`Amcl`], when to update, when to resample, when to publish |
//!
//! The scan model is a trait so a learned one (a network that scores a scan against the
//! map) can stand in for the likelihood field later, with nothing else changed.
#![warn(missing_docs, clippy::missing_errors_doc, clippy::must_use_candidate)]

pub mod amcl;
pub mod filter;
pub mod map;
pub mod motion;
pub mod odom;
pub mod pose;
pub mod rng;
pub mod scan;

pub use amcl::{Amcl, AmclSettings, SettingsError, Step};
pub use filter::{Estimate, FilterSettings, Particle, ParticleFilter};
pub use map::{Cell, DistanceField, FreeSpace, MapError, OccupancyMap};
pub use motion::DifferentialMotion;
pub use odom::OdomBuffer;
pub use pose::{wrap, Metres, Pose2, Radians};
pub use rng::Rng;
pub use scan::{Beam, LikelihoodField, Scan, ScanModel};
