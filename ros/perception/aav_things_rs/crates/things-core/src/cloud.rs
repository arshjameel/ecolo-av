//! Named columns of a `sensor_msgs/PointCloud2`'s bytes, any number type, as `f64`: what
//! `sensor_msgs_py`'s `read_points` gives for mixed fields (the radar clusters' x, y,
//! velocity float32 and cluster uint32). The reading is the shared `cloud-view` crate's
//! (TODO 176).

pub use cloud_view::{read_columns, CloudError, CloudView, Field};
