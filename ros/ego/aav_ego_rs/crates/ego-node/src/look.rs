//! The node `look_ahead`: messages in and out of ego-core's look_ahead (look_ahead_node.py's
//! twin), with a small TF tree of its own (`frames`).

use std::any::Any;
use std::time::Duration;

use anyhow::{bail, Result};
use ego_core::frames::{Transform, Tree};
use ego_core::look_ahead::{
    self as la, crop_ahead, extend, first_hit, gap, grade, nearest_index, to_goal, yaw_of, Frame2D,
    Grid, Level, Limits, Point, Pose2,
};
use ego_core::road_ahead::Strip;
use rclrs::*;
use ros_env::aav_interfaces::msg::RoadAhead;
use ros_env::diagnostic_msgs::msg::{DiagnosticStatus, KeyValue};
use ros_env::geometry_msgs::msg::{PoseStamped, Quaternion};
use ros_env::nav_msgs::msg::{OccupancyGrid, Path};
use ros_env::tf2_msgs::msg::TFMessage;

use crate::ego::strip_of;
use crate::params::Declarer;

pub struct Running {
    _keep: Vec<Box<dyn Any>>,
}

/// A message and when it arrived (ns, this node's clock).
type Kept<T> = Option<(T, i64)>;

struct Look {
    ahead_m: f64,
    /// s.
    local_timeout: f64,
    stale: f64,
    limits: Limits,
    look_m: f64,
    lethal: i32,
    base: String,
    front: f64,
    rear: f64,
    half_width: f64,
    trace: Kept<Path>,
    stop: Kept<PoseStamped>,
    local: Kept<Path>,
    plan: Kept<Path>,
    costmap: Kept<OccupancyGrid>,
    /// The road ahead, its frame, when it arrived.
    road: Option<(Strip, String, i64)>,
    tf: Tree,
    pub_plan: Publisher<Path>,
    pub_ahead: Publisher<DiagnosticStatus>,
    clock: Clock,
    logger: Logger,
}

fn quat_yaw(q: &Quaternion) -> f64 {
    yaw_of(q.x, q.y, q.z, q.w)
}

pub fn start(node: &Node) -> Result<Running> {
    let mut d = Declarer::new(node);
    let rate = d.number("rate_hz", 10.0)?;
    if rate <= 0.0 {
        bail!("rate_hz:={rate}: must be above 0");
    }
    let ahead_m = d.number("plan_ahead_m", 10.0)?;
    let local_timeout = d.number("local_plan_timeout", 0.5)?;
    let stale = d.number("stale_after", 1.0)?;
    let limits = Limits {
        warn: d.number("gap_warn", 0.3)?,
        bad: d.number("gap_bad", 0.6)?,
    };
    let look_m = d.number("look_m", 15.0)?;
    let lethal = i32::try_from(d.integer("lethal", 100)?)?;
    let base = d.text("base_frame", "base_link")?;
    // The Python reads vehicle.yaml's outline itself; this one gets it from the launch file.
    let Some(body) = d.numbers("body")? else {
        bail!(
            "body:=[front, rear, half width] (m from base_link, vehicle.yaml's outline) is \
             missing: ego.launch.py passes it"
        );
    };
    let [front, rear, half_width] = body[..] else {
        bail!("body:={body:?}: three numbers, [front, rear, half width]");
    };
    let look = Look {
        ahead_m,
        local_timeout,
        stale,
        limits,
        look_m,
        lethal,
        base,
        front,
        rear,
        half_width,
        trace: None,
        stop: None,
        local: None,
        plan: None,
        costmap: None,
        road: None,
        tf: Tree::new(),
        pub_plan: node.create_publisher("ego/plan_ahead")?,
        pub_ahead: node.create_publisher("ego/ahead")?,
        clock: node.get_clock(),
        logger: node.logger().clone(),
    };
    let mut keep: Vec<Box<dyn Any>> = Vec::new();
    let worker = node.create_worker(look);
    keep.push(Box::new(
        worker.create_subscription("ego/trace", |s: &mut Look, msg: Path| {
            s.trace = Some((msg, s.now()))
        })?,
    ));
    keep.push(Box::new(worker.create_subscription(
        "ego/stop_point",
        |s: &mut Look, msg: PoseStamped| s.stop = Some((msg, s.now())),
    )?));
    keep.push(Box::new(worker.create_subscription(
        "drivers/nav/local_plan",
        |s: &mut Look, msg: Path| s.local = Some((msg, s.now())),
    )?));
    keep.push(Box::new(
        worker.create_subscription("plan", |s: &mut Look, msg: Path| {
            s.plan = Some((msg, s.now()))
        })?,
    ));
    keep.push(Box::new(worker.create_subscription(
        "/drivers/nav/costmap/local",
        |s: &mut Look, msg: OccupancyGrid| s.costmap = Some((msg, s.now())),
    )?));
    keep.push(Box::new(worker.create_subscription(
        "world/road_ahead",
        |s: &mut Look, msg: RoadAhead| {
            let frame = if msg.header.frame_id.is_empty() {
                "map".to_string()
            } else {
                msg.header.frame_id.clone()
            };
            s.road = Some((strip_of(&msg), frame, s.now()));
        },
    )?));
    // As tf2's listener: /tf keep last 100; /tf_static latched.
    keep.push(Box::new(worker.create_subscription(
        "/tf".keep_last(100).reliable(),
        |s: &mut Look, msg: TFMessage| s.on_tf(&msg),
    )?));
    keep.push(Box::new(worker.create_subscription(
        "/tf_static".keep_last(100).reliable().transient_local(),
        |s: &mut Look, msg: TFMessage| s.on_tf(&msg),
    )?));
    keep.push(Box::new(worker.create_timer_repeating(
        Duration::from_secs_f64(1.0 / rate).node_time(),
        |s: &mut Look| s.tick(),
    )?));
    keep.push(Box::new(worker));
    keep.extend(d.finish());
    Ok(Running { _keep: keep })
}

/// 'nan' for none or not finite, else two decimals.
fn num(v: Option<f64>) -> String {
    match v {
        Some(v) if v.is_finite() => format!("{v:.2}"),
        _ => "nan".to_string(),
    }
}

impl Look {
    fn now(&self) -> i64 {
        self.clock.now().nsec
    }

    fn on_tf(&mut self, msg: &TFMessage) {
        for t in &msg.transforms {
            let r = &t.transform;
            self.tf.set(
                &t.header.frame_id,
                &t.child_frame_id,
                Transform {
                    translation: [r.translation.x, r.translation.y, r.translation.z],
                    rotation: [r.rotation.x, r.rotation.y, r.rotation.z, r.rotation.w],
                },
            );
        }
    }

    /// The message if it came within `within` s (`stale_after` by default).
    fn fresh<'a, T>(&self, kept: &'a Kept<T>, within: Option<f64>) -> Option<&'a T> {
        let (msg, at) = kept.as_ref()?;
        let age = (self.now() - at) as f64 * 1e-9;
        (age <= within.unwrap_or(self.stale)).then_some(msg)
    }

    /// frame -> base_link, the newest TF; `None` when unknown.
    fn frame_to_base(&self, frame: &str) -> Option<Frame2D> {
        if frame.is_empty() || frame == self.base {
            return Some(Frame2D::default());
        }
        let t = self.tf.lookup(&self.base, frame)?;
        Some(Frame2D::new(t.translation[0], t.translation[1], t.yaw()))
    }

    fn path_in_base(&self, msg: &Path) -> Option<Vec<Point>> {
        let f = self.frame_to_base(&msg.header.frame_id)?;
        Some(
            msg.poses
                .iter()
                .map(|ps| f.point(ps.pose.position.x, ps.pose.position.y))
                .collect(),
        )
    }

    fn grid_in_base(&self, msg: &OccupancyGrid) -> Option<Grid> {
        let f = self.frame_to_base(&msg.header.frame_id)?;
        let o = &msg.info.origin;
        let origin = f.pose(o.position.x, o.position.y, quat_yaw(&o.orientation));
        Some(Grid {
            width: msg.info.width as usize,
            height: msg.info.height as usize,
            resolution: f64::from(msg.info.resolution),
            origin,
            data: msg.data.clone(),
        })
    }

    /// The road ahead at the car; `None` when stale or unplaced.
    fn road_here(&self) -> Option<la::Road> {
        let (strip, frame, at) = self.road.as_ref()?;
        if (self.now() - at) as f64 * 1e-9 > self.stale {
            return None;
        }
        let car = self.tf.lookup(frame, &self.base)?;
        la::road(Some(strip), car.translation[0], car.translation[1])
    }

    fn tick(&mut self) {
        let Some(trace) = self.fresh(&self.trace, None) else {
            return;
        };
        let trace_xy: Vec<Point> = trace
            .poses
            .iter()
            .map(|ps| (ps.pose.position.x, ps.pose.position.y))
            .collect();
        let trace_poses: Vec<Pose2> = trace
            .poses
            .iter()
            .map(|ps| {
                (
                    ps.pose.position.x,
                    ps.pose.position.y,
                    quat_yaw(&ps.pose.orientation),
                )
            })
            .collect();
        let header_stamp = trace.header.stamp.clone();
        // Nav2 replans each second.
        let global_xy = self
            .fresh(&self.plan, Some(self.stale.max(3.0)))
            .and_then(|m| self.path_in_base(m));
        let local_xy = self
            .fresh(&self.local, Some(self.local_timeout))
            .filter(|m| !m.poses.is_empty())
            .and_then(|m| self.path_in_base(m));
        let (source, faint) = match (&local_xy, &global_xy) {
            (Some(l), _) if !l.is_empty() => ("local", crop_ahead(l, self.ahead_m)),
            (_, Some(g)) if g.len() >= 2 => {
                let reach = self.ahead_m.max(la::length(&trace_xy) + 2.0);
                ("global", crop_ahead(g, reach))
            }
            _ => ("none", Vec::new()),
        };
        let gap_m = gap(&trace_xy, &faint);
        let level = grade(gap_m, &self.limits);

        let stop_xy = self
            .fresh(&self.stop, None)
            .map(|s| (s.pose.position.x, s.pose.position.y));
        let stop_m = stop_xy.map(|(x, y)| ego_core::hypot(x, y));
        let goal_m = match (stop_xy, &global_xy) {
            (Some(stop), Some(g)) if g.len() >= 2 => to_goal(&g[nearest_index(g)..], stop),
            _ => None,
        };
        let grid = self
            .fresh(&self.costmap, Some(self.stale.max(2.0)))
            .and_then(|m| self.grid_in_base(m));
        let obstacle_m = grid.as_ref().and_then(|g| {
            first_hit(
                g,
                &extend(&trace_poses, self.look_m),
                self.front,
                self.half_width,
                self.lethal,
                0.1,
            )
        });
        let room_m = match (obstacle_m, stop_m) {
            (Some(o), Some(s)) => Some(o - s),
            _ => None,
        };

        let mut out = Path::default();
        out.header.stamp = header_stamp;
        out.header.frame_id = self.base.clone();
        out.poses = faint
            .iter()
            .map(|&(x, y)| {
                let mut ps = PoseStamped {
                    header: out.header.clone(),
                    ..Default::default()
                };
                ps.pose.position.x = x;
                ps.pose.position.y = y;
                ps.pose.orientation.w = 1.0;
                ps
            })
            .collect();
        if let Err(e) = self.pub_plan.publish(out) {
            log_error!(&self.logger, "ego/plan_ahead: {e}");
        }

        let road = self.road_here();
        let values = [
            ("gap_m", num(gap_m)),
            ("plan", source.to_string()),
            ("stop_m", num(stop_m)),
            ("goal_m", num(goal_m)),
            ("obstacle_m", num(obstacle_m)),
            ("room_m", num(room_m)),
            ("front_m", num(Some(self.front))),
            ("rear_m", num(Some(self.rear))),
            ("half_width_m", num(Some(self.half_width))),
            ("road_mps", num(road.as_ref().map(|r| r.limit))),
            ("road_m", num(road.as_ref().map(|r| r.room))),
            ("road_why", road.map_or(String::new(), |r| r.why)),
        ];
        let status = DiagnosticStatus {
            level: level_byte(level),
            name: "look_ahead".to_string(),
            message: la::describe(gap_m, level, stop_m, goal_m, room_m),
            hardware_id: "aav_ego".to_string(),
            values: values
                .into_iter()
                .map(|(k, v)| KeyValue {
                    key: k.to_string(),
                    value: v,
                })
                .collect(),
        };
        if let Err(e) = self.pub_ahead.publish(status) {
            log_error!(&self.logger, "ego/ahead: {e}");
        }
    }
}

fn level_byte(level: Level) -> u8 {
    level.byte()
}
