//! The map window's logic, with no ROS and no raylib, so it is tested on any laptop.
//!
//! The map window (`map`, in hud-app's workspace) is the big screen's program: Nav2's
//! map, the car on it, the planned path and the goal, and how far is left.
//!
//! | Module | What it holds |
//! |---|---|
//! | [`config`] | the topic names, from `config/map.yaml` |
//! | [`earth`] | latitude and longitude to the map frame and back, as navsat_transform does |
//! | [`geom`] | points and poses in the map frame (metres, radians) |
//! | [`grid`] | Nav2's occupancy grid, and its pixels for the screen |
//! | [`route`] | the planned path: the nearest point to the car, the distance left |
//! | [`car`] | the car's outline, from `vehicle.yaml` |
//! | [`robot`] | the car as the little robot (`car_marker: robot`), on its true outline |
//! | [`view`] | map to screen and back: follow the car, the whole map, zoom, the scale bar |
//! | [`trip`] | NO GOAL, DRIVING, ARRIVED... and the time left |
//! | [`tiles`] | the street map's tiles: which ones the view needs, at which zoom |
//! | [`pmtiles`] | reads the street map's file (PMTiles) |
//! | [`streets`] | one tile's water, parks, buildings and roads, as triangles and lines in map metres |
//! | [`tilecache`] | the decoded tiles: what to draw, what to load, what to forget |
//! | [`places`] | the place search: names and addresses from `ottawa.db` (SQLite) |
//! | [`destination`] | the search box, picking a place, and whether it can be Nav2's goal |
//! | [`keyboard`] | the on-screen keyboard for the search: where its keys are, which one a tap hit |
//! | [`state`] | everything above, as one value the screen draws |

pub mod car;
pub mod config;
pub mod destination;
pub mod earth;
pub mod geom;
pub mod grid;
pub mod keyboard;
pub mod places;
pub mod pmtiles;
pub mod robot;
pub mod route;
pub mod state;
pub mod streets;
pub mod tilecache;
pub mod tiles;
pub mod trip;
pub mod view;
