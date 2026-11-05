#!/usr/bin/env python3
"""
Save a picture of the running ROS graph: every node and topic, and who talks to whom.

    pixi run tools/ros.sh python3 tools/graph_picture.py
        docs/img/graph.svg and .png (start a profile first)
    tools/ros.sh python3 tools/graph_picture.py --out /tmp/graph.svg [--wait 4] [--all]

Like rqt_graph's Nodes/Topics view, but in a file, and grouped by layer: sensors at the
left, the backend at the right. It leaves out ROS's own plumbing (rosout, parameters,
actions, tf, Nav2's internal topics); --all keeps everything.
"""

import argparse
from pathlib import Path
import re
import subprocess
import time

import rclpy

# Which layer each node belongs to: (column title, fill colour). The order is left to right.
LAYERS = [
    ('Hardware stand-ins (bench)', '#F2F2F2',
     r'^(spine_sim|mock_.*|test_camera|fake_lane_geometry_pub)$'),
    ('Sensors', '#FDECEE',
     r'^(imu_node|imu_filter_madgwick|gps_nmea|ublox.*|radar|battery_monitor|camera_.*)$'),
    ('Perception', '#FFF4D6',
     r'^(object_detector(_\w+)?|stop_sign_watcher|radar_clusters|control_interface)$'),
    ('Localization', '#E8F1FB', r'^(ekf_local|ekf_global|navsat_transform)$'),
    ('Drivers', '#EAF6EA',
     r'^(scripted_driver|joy_node|teleop_twist_joy_node|joy_teleop|teleop_twist_keyboard'
     r'|bt_navigator|controller_server|planner_server|behavior_server|velocity_smoother'
     r'|waypoint_follower|smoother_server|map_server|collision_monitor)$'),
    ('HUD', '#E8F1FB', r'^(hud|map)$'),
    ('Mux and ego', '#F6E8F6', r'^(twist_mux|ego)$'),
    ('Backend', '#FDECEE', r'^(spine_bridge|legacy_.*|encoder_udp|.*_adapter|socket_can_.*)$'),
    ('Car model', '#F2F2F2', r'^(robot_state_publisher|joint_state_publisher)$'),
]
HIDDEN_NODES = r'^(_ros2cli.*|launch_ros_.*|transform_listener_impl_.*|graph_picture|rviz2|' \
               r'lifecycle_manager.*|.*_rclcpp_node|.*_client_node|global_costmap|local_costmap)$'
HIDDEN_TOPICS = r'(^/rosout$|^/parameter_events$|/_action/|^/tf(_static)?$|^/clock$|/bond$|' \
                r'transition_event$|costmap|^/behavior_tree_log$|^/plan_smoothed$|' \
                r'^/(speed_limit|cmd_vel_teleop|lookahead_.*|received_global_plan|' \
                r'unsmoothed_plan|preempt_teleop|cmd_vel_nav|evaluation|marker|local_plan|' \
                r'transformed_global_plan|waypoints|cost_cloud|polygon_.*|diagnostics|' \
                r'is_rotating_to_heading|curvature_lookahead_point)$|' \
                r'/(describe|get|list|set)_parameters|/robot_description$|/joint_states$)'


def layer_of(name):
    for i, (_, _, pattern) in enumerate(LAYERS):
        if re.match(pattern, name):
            return i
    return len(LAYERS)


def quote(s):
    return '"' + s.replace('"', r'\"') + '"'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=str(Path(__file__).resolve().parents[1] / 'docs' / 'img'
                                         / 'graph.svg'))
    ap.add_argument('--wait', type=float, default=15.0,
                    help='seconds to let discovery settle: a big graph needs about 15')
    ap.add_argument('--all', action='store_true', help="keep ROS's own topics too")
    args = ap.parse_args()

    rclpy.init()
    probe = rclpy.create_node('graph_picture')
    end = time.monotonic() + args.wait
    while time.monotonic() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)

    nodes, edges, topics = {}, set(), {}
    for name, ns in probe.get_node_names_and_namespaces():
        if not args.all and re.match(HIDDEN_NODES, name):
            continue
        full = (ns.rstrip('/') + '/' + name) if ns != '/' else name
        nodes[full] = layer_of(name)
        for kind, get in (('pub', probe.get_publisher_names_and_types_by_node),
                          ('sub', probe.get_subscriber_names_and_types_by_node)):
            for topic, types in get(name, ns):
                if not args.all and re.search(HIDDEN_TOPICS, topic):
                    continue
                topics[topic] = types[0].split('/')[-1]
                edges.add((full, topic) if kind == 'pub' else (topic, full))
    probe.destroy_node()
    rclpy.try_shutdown()

    # Keep only topics with both ends in the picture, or a publisher with no listener yet.
    used = {t for t in topics if any(a == t or b == t for a, b in edges)}
    lines = ['digraph aav {', '  rankdir=LR; newrank=true; splines=true; nodesep=0.15;',
             '  ranksep=0.9; fontname="Helvetica";',
             '  node [fontname="Helvetica", fontsize=11]; edge [color="#888888"];']
    # An invisible ladder holds the layers in order, left to right: each layer's nodes share
    # a rank with one rung (newrank lets rank=same reach into clusters).
    shown = [i for i in range(len(LAYERS) + 1) if any(v == i for v in nodes.values())]
    lines.append('  ' + ' -> '.join(f'rung{i}' for i in shown)
                 + ' [style=invis, weight=50];')
    for i in shown:
        title, colour, _ = (LAYERS + [('Other', '#FFFFFF', '')])[i]
        members = sorted(n for n, layer in nodes.items() if layer == i)
        lines.append(f'  rung{i} [style=invis, shape=point, width=0];')
        lines.append(f'  subgraph cluster_{i} {{ label={quote(title)}; fontsize=13; '
                     f'color="#155E8C"; style="rounded";')
        for n in members:
            lines.append(f'    {quote(n)} [shape=box, style="rounded,filled", '
                         f'fillcolor={quote(colour)}, color="#155E8C"];')
        lines.append('  }')
        lines.append('  { rank=same; ' + f'rung{i}; ' + ' '.join(quote(n) for n in members)
                     + ' }')
    for t in sorted(used):
        lines.append(f'  {quote(t)} [shape=plaintext, fontsize=9, fontcolor="#222222", '
                     f'label={quote(t + chr(10) + topics[t])}];')
    for a, b in sorted(edges):
        if a in nodes or b in nodes:
            lines.append(f'  {quote(a)} -> {quote(b)};')
    lines.append('}')

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    dot = out.with_suffix('.dot')
    dot.write_text('\n'.join(lines) + '\n')
    for fmt in ('svg', 'png'):
        subprocess.run(['dot', f'-T{fmt}', '-Gdpi=110', str(dot), '-o',
                        str(out.with_suffix('.' + fmt))], check=True)
    dot.unlink()
    print(f'{len(nodes)} nodes, {len(used)} topics -> {out.with_suffix(".svg")} and .png')


if __name__ == '__main__':
    main()
