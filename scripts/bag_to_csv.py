#!/usr/bin/env python3
"""Converts a benchmark rosbag (mcap) into the CSV files shipped with each case.

Pure Python, no ROS install needed:
    pip install rosbags numpy
    python3 scripts/bag_to_csv.py data/case01_fold_clothes_spd45

Outputs (next to the bag directory):
    eef_command.csv  dual-arm end-effector command in BASE frame, one row per tick
                     with left (l_*) and right (r_*) poses
    meta.json        duration and start pose; full statistics are printed to stdout
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

ARMS = {
    'l': 'left_gripper_f90c_end_effector_link',
    'r': 'right_gripper_f90c_end_effector_link',
}
MSG_DIR = Path(__file__).resolve().parent.parent / 'msg' / 'ros2_msg' / 'msg'
# Both arms are commanded in the same control tick; anything wider means the
# rows would pair poses from different ticks.
MAX_ARM_STAMP_SKEW_NS = 1_000_000


def command_topic(link):
    return f'/end_command_in_base/{link}'


def make_typestore():
    store = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for name in ('LinkInfoPose', 'LinkInfo'):
        types.update(get_types_from_msg((MSG_DIR / f'{name}.msg').read_text(),
                                        f'ros2_msg/msg/{name}'))
    store.register(types)
    return store


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def read_commands(bag_dir):
    """Returns {arm: [(stamp_ns, x, y, z, qx, qy, qz, qw), ...]}."""
    routes = {command_topic(link): arm for arm, link in ARMS.items()}
    commands = {arm: [] for arm in ARMS}
    with AnyReader([bag_dir], default_typestore=make_typestore()) as reader:
        conns = [c for c in reader.connections if c.topic in routes]
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            commands[routes[conn.topic]].append(
                (stamp_ns(msg), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    return commands


def sorted_command(command):
    stamps = np.array([m[0] for m in command], dtype=np.int64)
    values = np.array([m[1:] for m in command], dtype=float)
    order = np.argsort(stamps, kind='stable')
    return stamps[order], values[order]


def trim_unpaired_ends(left, right):
    """Drops samples at either end that have no partner from the other arm.

    Recording can start or stop between the two arms' publishes of one tick, so
    one arm may carry an extra sample at the edges. Gaps in the middle are not
    touched here; they fail the count/skew checks instead.
    """
    (sl, vl), (sr, vr) = left, right
    dropped = {'l': 0, 'r': 0}
    while len(sl) and len(sr) and abs(sl[0] - sr[0]) > MAX_ARM_STAMP_SKEW_NS:
        arm = 'l' if sl[0] < sr[0] else 'r'
        if arm == 'l':
            sl, vl = sl[1:], vl[1:]
        else:
            sr, vr = sr[1:], vr[1:]
        dropped[arm] += 1
    while len(sl) and len(sr) and (len(sl) != len(sr)
                                   or abs(sl[-1] - sr[-1]) > MAX_ARM_STAMP_SKEW_NS):
        if sl[-1] > sr[-1]:
            sl, vl = sl[:-1], vl[:-1]
            dropped['l'] += 1
        else:
            sr, vr = sr[:-1], vr[:-1]
            dropped['r'] += 1
    return (sl, vl), (sr, vr), dropped


def make_quaternions_continuous(quat):
    """Flips signs so consecutive quaternions stay in the same hemisphere."""
    quat = quat / np.linalg.norm(quat, axis=1, keepdims=True)
    for i in range(1, len(quat)):
        if np.dot(quat[i], quat[i - 1]) < 0.0:
            quat[i] = -quat[i]
    return quat


def angular_speed(quat, dt):
    """|omega| from consecutive unit quaternions (xyzw)."""
    dots = np.abs(np.sum(quat[1:] * quat[:-1], axis=1)).clip(max=1.0)
    return 2.0 * np.arccos(dots) / dt


def stats(x):
    return {'p50': float(np.percentile(x, 50)), 'p99': float(np.percentile(x, 99)),
            'max': float(x.max())}


def summarize_arm(t, pos, quat):
    # Resample at the native median period: a finer grid differentiates the
    # linear interpolation kinks and inflates acceleration.
    step = float(np.median(np.diff(t)))
    grid = np.arange(t[0], t[-1], step)
    pos_u = np.column_stack([np.interp(grid, t, pos[:, k]) for k in range(3)])
    quat_u = np.column_stack([np.interp(grid, t, quat[:, k]) for k in range(4)])
    quat_u /= np.linalg.norm(quat_u, axis=1, keepdims=True)
    vel = np.gradient(pos_u, step, axis=0)
    acc = np.gradient(vel, step, axis=0)
    return {
        'position_min_m': pos.min(axis=0).tolist(),
        'position_max_m': pos.max(axis=0).tolist(),
        'linear_speed_mps': stats(np.linalg.norm(vel, axis=1)),
        'linear_acc_mps2': stats(np.linalg.norm(acc, axis=1)),
        'angular_speed_radps': stats(angular_speed(quat_u, step)),
        'start_pose_xyz_qxyzw': [*pos[0].tolist(), *quat[0].tolist()],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('case_dir', type=Path, help='case directory containing bag/')
    parser.add_argument('--start-s', type=float, default=0.0,
                        help='drop commands before this many seconds into the recording, '
                             'e.g. a startup transient that is not part of the test')
    args = parser.parse_args()

    commands = read_commands(args.case_dir / 'bag')
    stamps_l, values_l = sorted_command(commands['l'])
    stamps_r, values_r = sorted_command(commands['r'])
    (stamps_l, values_l), (stamps_r, values_r), dropped = trim_unpaired_ends(
        (stamps_l, values_l), (stamps_r, values_r))
    if len(stamps_l) != len(stamps_r):
        raise SystemExit(f'left/right command counts differ: {len(stamps_l)} vs {len(stamps_r)}')
    skew = np.abs(stamps_l - stamps_r).max()
    if skew > MAX_ARM_STAMP_SKEW_NS:
        raise SystemExit(f'left/right command stamps drift apart by {skew * 1e-6:.3f} ms')

    stamps = stamps_l
    raw_t = (stamps - stamps[0]) * 1e-9
    start = int(np.searchsorted(raw_t, args.start_s))
    stamps = stamps[start:]
    t0 = stamps[0]
    t = (stamps - t0) * 1e-9
    pos = {'l': values_l[start:, :3], 'r': values_r[start:, :3]}
    quat = {'l': make_quaternions_continuous(values_l[start:, 3:]),
            'r': make_quaternions_continuous(values_r[start:, 3:])}

    pose_cols = ['x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw']
    with open(args.case_dir / 'eef_command.csv', 'w', newline='') as f:
        w = csv.writer(f, lineterminator='\n')
        w.writerow(['t_s', 'stamp_ns', *(f'{arm}_{c}' for arm in ARMS for c in pose_cols)])
        for i in range(len(t)):
            row = [f'{t[i]:.6f}', stamps[i]]
            for arm in ARMS:
                row += [f'{v:.9f}' for v in (*pos[arm][i], *quat[arm][i])]
            w.writerow(row)

    # meta.json holds only what the tester needs to set up a run; the full
    # statistics go to stdout for whoever prepares the case.
    arms = {arm: summarize_arm(t, pos[arm], quat[arm]) for arm in ARMS}
    meta = {
        'duration_s': round(float(t[-1]), 3),
        'start_pose_in_base': {
            arm: {'xyz_m': [round(v, 6) for v in arms[arm]['start_pose_xyz_qxyzw'][:3]],
                  'qxyzw': [round(v, 6) for v in arms[arm]['start_pose_xyz_qxyzw'][3:]]}
            for arm in ARMS},
    }
    (args.case_dir / 'meta.json').write_text(json.dumps(meta, indent=2) + '\n')

    dt = np.diff(t)
    distance = np.linalg.norm(pos['l'] - pos['r'], axis=1)
    report = {
        'samples': int(len(t)),
        'sample_period_s': {'median': float(np.median(dt)), 'min': float(dt.min()),
                            'max': float(dt.max())},
        'max_arm_stamp_skew_ms': float(skew * 1e-6),
        'dropped_unpaired_edge_samples': dropped,
        'trimmed_start_s': float(raw_t[start]),
        'inter_arm_distance_m': {'min': float(distance.min()),
                                 'p1': float(np.percentile(distance, 1)),
                                 'median': float(np.median(distance))},
        'arms': arms,
    }
    print(json.dumps(report, indent=2))

if __name__ == '__main__':
    main()
