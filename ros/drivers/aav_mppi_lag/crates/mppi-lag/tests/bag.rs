//! `follow_plan`'s recording reader (examples/bag, TODO 202): plans written into an MCAP
//! as rosbag2 writes them (CDR, zstd chunks) come back pose for pose, in order.

#[path = "../examples/bag/mod.rs"]
mod bag;

use std::collections::BTreeMap;

use mcap::records::MessageHeader;
use mppi_lag::Pose;

fn arc(n: usize, radius: f64) -> Vec<Pose> {
    (0..n)
        .map(|i| {
            let a = i as f64 * 0.1;
            Pose::new(radius * a.sin(), radius * (1.0 - a.cos()), a)
        })
        .collect()
}

fn close(a: &[Pose], b: &[Pose]) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(p, q)| {
            (p.x - q.x).abs() < 1e-12 && (p.y - q.y).abs() < 1e-12 && (p.yaw - q.yaw).abs() < 1e-9
        })
}

#[test]
fn a_path_survives_cdr_both_ways() {
    let poses = arc(30, 5.0);
    let (frame, back) = bag::decode_path(&encode_path("map", &poses)).expect("a path");
    assert_eq!(frame, "map");
    assert!(close(&poses, &back));
}

#[test]
fn a_cut_message_is_an_error_not_a_panic() {
    let bytes = encode_path("map", &arc(5, 5.0));
    for end in [0, 3, 10, bytes.len() / 2, bytes.len() - 1] {
        assert!(bag::decode_path(&bytes[..end]).is_err(), "cut at {end}");
    }
}

#[test]
fn the_plans_on_one_topic_come_back_in_order_and_others_are_left_out() {
    let mut file = std::io::Cursor::new(Vec::new());
    let mut writer = mcap::Writer::new(&mut file).expect("a writer");
    let schema = writer
        .add_schema("nav_msgs/msg/Path", "ros2msg", b"")
        .expect("a schema");
    let meta = BTreeMap::new();
    let plan = writer
        .add_channel(schema, "/plan", "cdr", &meta)
        .expect("a channel");
    let other = writer
        .add_channel(schema, "/drivers/nav/local_plan", "cdr", &meta)
        .expect("a channel");
    let paths = [arc(10, 5.0), arc(20, 8.0), arc(5, 3.0)];
    for (i, poses) in paths.iter().enumerate() {
        for (channel, offset) in [(plan, 0), (other, 1)] {
            let time = 1_000_000_000 * (i as u64 + 1) + offset;
            let header = MessageHeader {
                channel_id: channel,
                sequence: i as u32,
                log_time: time,
                publish_time: time,
            };
            writer
                .write_to_known_channel(&header, &encode_path("map", poses))
                .expect("written");
        }
    }
    writer.finish().expect("finished");
    drop(writer);

    let found = bag::plans(file.get_ref(), "/plan").expect("read");
    assert_eq!(found.len(), 3);
    for (i, (plan, poses)) in found.iter().zip(&paths).enumerate() {
        assert!(close(&plan.poses, poses), "plan {i}");
        assert_eq!(plan.frame, "map");
        assert!((plan.time - i as f64).abs() < 1e-9);
    }
}

/// A `nav_msgs/Path` as little-endian CDR, as rclcpp sends it (for the tests).
fn encode_path(frame: &str, poses: &[Pose]) -> Vec<u8> {
    fn pad(out: &mut Vec<u8>, to: usize) {
        while !(out.len() - 4).is_multiple_of(to) {
            out.push(0);
        }
    }
    fn string(out: &mut Vec<u8>, s: &str) {
        pad(out, 4);
        out.extend_from_slice(&(s.len() as u32 + 1).to_le_bytes());
        out.extend_from_slice(s.as_bytes());
        out.push(0);
    }
    fn header(out: &mut Vec<u8>, frame: &str) {
        pad(out, 4);
        out.extend_from_slice(&[0; 8]);
        string(out, frame);
    }
    let mut out = vec![0, 1, 0, 0];
    header(&mut out, frame);
    pad(&mut out, 4);
    out.extend_from_slice(&(poses.len() as u32).to_le_bytes());
    for p in poses {
        header(&mut out, frame);
        let (s, c) = (p.yaw / 2.0).sin_cos();
        for v in [p.x, p.y, 0.0, 0.0, 0.0, s, c] {
            pad(&mut out, 8);
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
    out
}
