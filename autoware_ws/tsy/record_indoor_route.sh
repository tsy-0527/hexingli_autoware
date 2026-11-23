#!/usr/bin/env bash
# Run in the sourced ROS 2 / Autoware environment on the vehicle computer.
# Usage: bash record_indoor_route.sh [output_bag_directory]
set -euo pipefail
if [[ $# -gt 1 ]]; then
  printf '%s\n' 'Usage: bash record_indoor_route.sh [output_bag_directory]' >&2
  exit 1
fi
for cmd in ros2 python3; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    printf 'ERROR: %s not found. Source ROS 2 and Autoware setup.bash first.\n' "$cmd" >&2
    exit 1
  fi
done
output="${1:-indoor_route_$(date +%Y%m%d_%H%M%S)}"
output="$(python3 -c 'import os,sys; print(os.path.abspath(os.path.expanduser(sys.argv[1])))' "$output")"
if [[ -e "$output" ]]; then
  printf 'ERROR: output already exists: %s\n' "$output" >&2
  exit 1
fi
parent="$(dirname "$output")"
if [[ ! -d "$parent" || ! -w "$parent" ]]; then
  printf 'ERROR: parent directory must exist and be writable: %s\n' "$parent" >&2
  exit 1
fi
printf 'Bag output: %s\nROS_DISTRO: %s\nROS_DOMAIN_ID: %s\n' "$output" "${ROS_DISTRO:-not set}" "${ROS_DOMAIN_ID:-0 (default)}"
printf '%s\n' 'Checking live /tf and /localization/kinematic_state for up to 12 seconds...'
python3 - <<'PY'
import math
import sys
import time
try:
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry
except ImportError as exc:
    sys.exit(f'ERROR: ROS Python dependency unavailable: {exc}. Source the ROS/Autoware environment.')

rclpy.init()
node = rclpy.create_node('indoor_route_record_precheck')
qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)
state = {'tf_count': 0, 'odom_count': 0, 'tf_last': 0.0, 'odom_last': 0.0,
         'parents': set(), 'error': None}

def valid_pose(p, q):
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    return all(math.isfinite(v) for v in values) and abs(norm - 1.0) < 0.05

def on_tf(msg):
    for tf in msg.transforms:
        if tf.child_frame_id != 'base_link':
            continue
        state['parents'].add(tf.header.frame_id)
        if tf.header.frame_id != 'map':
            state['error'] = ('/tf contains a base_link transform with parent '
                              f'{tf.header.frame_id!r}; upstream bag2lanelet does not compose TF chains. '
                              'Use a converter that resolves map -> base_link; do not publish a duplicate TF.')
            continue
        if not valid_pose(tf.transform.translation, tf.transform.rotation):
            state['error'] = 'Invalid position/quaternion in map -> base_link.'
            continue
        state['tf_count'] += 1
        state['tf_last'] = time.monotonic()

def on_odom(msg):
    if msg.header.frame_id != 'map' or msg.child_frame_id != 'base_link':
        state['error'] = ('Odometry frames must be map / base_link for this recording workflow; got '
                          f'{msg.header.frame_id!r} / {msg.child_frame_id!r}.')
        return
    if not valid_pose(msg.pose.pose.position, msg.pose.pose.orientation):
        state['error'] = 'Invalid position/quaternion in localization odometry.'
        return
    state['odom_count'] += 1
    state['odom_last'] = time.monotonic()

subs = [node.create_subscription(TFMessage, '/tf', on_tf, qos),
        node.create_subscription(Odometry, '/localization/kinematic_state', on_odom, qos)]
start = time.monotonic()
success = False
try:
    while rclpy.ok() and time.monotonic() - start < 12.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic()
        if state['error']:
            break
        if (now - start >= 3.0 and state['tf_count'] >= 5 and state['odom_count'] >= 5
                and now - state['tf_last'] < 1.0 and now - state['odom_last'] < 1.0):
            success = True
            break
except KeyboardInterrupt:
    state['error'] = 'Preflight cancelled.'
finally:
    node.destroy_node()
    rclpy.shutdown()
if not success:
    print('ERROR: ' + (state['error'] or
          'Timed out waiting for repeated live map -> base_link TF and map-frame odometry.'), file=sys.stderr)
    print(f"Received valid TF: {state['tf_count']}; odometry: {state['odom_count']}; "
          f"base_link parents: {sorted(state['parents'])}", file=sys.stderr)
    print('Check Autoware localization, topic names, ROS_DOMAIN_ID and ROS environment.', file=sys.stderr)
    sys.exit(1)
print(f"PASS: live TF ({state['tf_count']} messages) and odometry ({state['odom_count']} messages).")
print('This checks message reception, frames and numeric validity only.')
print('Confirm localization alignment/stability in RViz; map projection/MGRS is NOT verified.')
PY
# Preserve latched static transforms for subsequent offline inspection.
qos_file="$(mktemp "${TMPDIR:-/tmp}/indoor_record_qos.XXXXXX")"
trap 'rm -f -- "$qos_file"' EXIT
printf '%s\n' '/tf_static:' '  reliability: reliable' '  durability: transient_local' '  history: keep_last' '  depth: 100' > "$qos_file"
printf '\nSaving to: %s\n' "$output"
printf '%s\n' 'Wait for the recorder to start before driving manually. Press Ctrl+C to finish.'
# Keep the wrapper alive so the temporary QoS file is removed on exit.
trap ':' INT
set +e
ros2 bag record --storage sqlite3 --output "$output" \
  --qos-profile-overrides-path "$qos_file" \
  /tf /tf_static /localization/kinematic_state
status=$?
set -e
printf '\nRecorder exited (code %s). Bag directory: %s\n' "$status" "$output"
if [[ -f "$output/metadata.yaml" ]]; then
  printf '%s\n' 'metadata.yaml exists. Inspect topic counts and duration using:'
  printf 'ros2 bag info "%s"\n' "$output"
else
  printf '%s\n' 'WARNING: metadata.yaml was not found. Recording may have failed or not finalized.' >&2
fi
exit "$status"
