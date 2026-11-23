# Cost-aware path proxy

This is an external ROS 2 sidecar for evaluating cost-based local avoidance without
modifying Autoware's behavior planner. It generates local `KEEP_LANE`, `LEFT_SHIFT`,
`RIGHT_SHIFT`, and `STOP_WAIT` candidates from the existing `PathWithLaneId`.

The default mode is `shadow`. The proxy publishes its selected path to an isolated
topic and does not replace any Autoware topic.

With the planning simulation design, the dummy-object path is:

```text
/simulation/dummy_perception_publisher/object_info   (DummyObject)
        -> dummy perception / tracking / prediction
/perception/object_recognition/objects                (PredictedObjects)
        -> cost_aware_path_proxy
```

Therefore the `DummyObject` publisher does not need to be modified for this test.

## Shadow mode

Start Autoware normally, then run:

```bash
./scripts/run_cost_aware_path_proxy.sh \
  --ros-args \
  -p mode:=shadow
```

Inspect the selected behavior and score breakdown:

```bash
ros2 topic echo \
  /planning/scenario_planning/lane_driving/behavior_planning/cost_aware/debug
```

Candidate paths are published under:

```text
/planning/scenario_planning/lane_driving/behavior_planning/cost_aware/candidate/keep_lane
/planning/scenario_planning/lane_driving/behavior_planning/cost_aware/candidate/left_shift
/planning/scenario_planning/lane_driving/behavior_planning/cost_aware/candidate/right_shift
/planning/scenario_planning/lane_driving/behavior_planning/cost_aware/candidate/stop_wait
```

The proxy uses an ego-anchored local path by default. It looks up
`map -> base_link`, removes path points behind the nearest point to the vehicle,
and anchors the first remaining point to the current vehicle pose. The debug message
contains `path_start.mode: "ego"` when this succeeds. If TF is unavailable or the
vehicle is too far from the input path, the original path is passed through and the
reason is reported in `path_start`.

Relevant parameters are:

```text
crop_path_to_ego              true
ego_frame                     base_link
max_ego_to_path_distance_m    5.0
ego_tf_timeout_sec            0.05
```

For the current simulation, `map -> base_link` must be available. Verify it with:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

For the already-running Autoware RViz, the selected candidate is also converted to
`nav_msgs/Path` and published at:

```text
/planning/scenario_planning/lane_driving/behavior_planning/cost_aware/rviz_path
```

The exact input path topic can differ between launch presets. Override it if needed:

```bash
./scripts/run_cost_aware_path_proxy.sh \
  --ros-args \
  -p input_path_topic:=/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
```

## Simulation test procedure

Use three terminals after the planning simulation and RViz are running.

Terminal 1 starts the existing click obstacle publisher. The repository already has
the corrected version:

```bash
cd /home/tang/autoware
source /opt/ros/humble/setup.bash
source /home/tang/autoware/install/setup.bash
python3 scripts/click_obstacle_fixed.py
```

If using your own publisher, save it as a `.py` file and fix the formatting artifacts
from the pasted snippet (`__init__`, `__name__`, and normal underscores) before running
it. Its `DummyObject` topic and `map` frame are compatible with this proxy.

Terminal 2 starts the cost-aware proxy:

```bash
cd /home/tang/autoware
./scripts/run_cost_aware_path_proxy.sh \
  --ros-args \
  -p mode:=shadow
```

The runner sources ROS 2 before enabling strict shell variable checking. This is
intentional because Humble's `setup.bash` may reference unset optional environment
variables such as `AMENT_TRACE_SETUP_FILES`.

Terminal 3 verifies that the dummy object reached the planning perception topic:

```bash
ros2 topic echo /perception/object_recognition/objects --once
ros2 topic echo \
  /planning/scenario_planning/lane_driving/behavior_planning/cost_aware/debug
```

To open a second, dedicated RViz view with the original path, selected path, all
candidates, predicted objects, map markers, and the checkpoint obstacle tool
preconfigured, run this in another terminal. The existing Autoware RViz can remain
open:

```bash
cd /home/tang/autoware
./scripts/run_cost_aware_rviz.sh
```

The selected path is drawn in red. The original `path_with_lane_id` is gray. The
left, right, keep-lane, and stop candidates are available in the Displays panel and
are initially disabled to keep the view readable.

If you want to use the existing Autoware RViz directly, add
`rviz_default_plugins/Path` and set its topic to the `cost_aware/rviz_path` topic
above. Set `Fixed Frame` to `map`, enable `View Path`, and choose a visible color
such as red. This display is isolated from Autoware's original path topic.

In RViz, the dummy object is normally shown by the existing `DummyObject` display.
The proxy path is not part of the preloaded RViz configuration, so add a new
`rviz_plugins/PathWithLaneId` display and set its topic to:

```text
/planning/scenario_planning/lane_driving/behavior_planning/cost_aware/path_with_lane_id
```

Keep the existing original path display enabled for comparison:

```text
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
```

For candidate comparison, add more `rviz_plugins/PathWithLaneId` displays for the four
`cost_aware/candidate/*` topics. Use different colors so the selected path can be
compared with the original, left, and right candidates.

Then use RViz's `2D Checkpoint Pose` tool to place an obstacle ahead of the vehicle.
Wait for the perception chain to publish the object, and inspect `selected` and the
candidate `total_cost` values in the debug topic. The selected path means the lowest
cost among the valid candidates generated by this prototype; it is not yet a global
Apollo-style optimizer result.

## Override mode

Only use this in simulation or a controlled test. It is opt-in:

```bash
./scripts/run_cost_aware_path_proxy.sh \
  --ros-args \
  -p mode:=override
```

The proxy output is:

```text
/planning/scenario_planning/lane_driving/behavior_planning/cost_aware/path_with_lane_id
```

For an actual pipeline test, remap the `behavior_velocity_planner` input from
`/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id` to
that output. This keeps the normal behavior-velocity stage after the proxy. Use a
launch overlay or local launch copy; do not publish the proxy output back onto the
original path topic because that would create two competing publishers.

## Main parameters

```text
planning_horizon_m              60.0
vehicle_width_m                  1.0
vehicle_length_m                 4.0
hard_clearance_m                 0.35
candidate_clearance_buffer_m     0.05
min_shift_m                      0.80
max_shift_m                      2.50
shift_prepare_distance_m         6.0
shift_length_m                   8.0
return_length_m                  8.0
weights.lateral                  2.0
weights.smoothness               8.0
weights.clearance                8.0
weights.risk                    20.0
weights.boundary                 8.0
switch_cost                      3.0
```

This prototype deliberately keeps lane IDs and turn-signal commands from the
original path. It is therefore suitable for validating local lateral avoidance, not
for validating a full lane-change or opposite-lane-borrow maneuver.
