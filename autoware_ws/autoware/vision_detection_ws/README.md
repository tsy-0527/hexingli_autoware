# Standalone camera detection workspace

This workspace starts a standard USB camera and Autoware's installed TensorRT
YOLOX detector without changing the Autoware source tree. It publishes 2D detections as
`tier4_perception_msgs/msg/DetectedObjectsWithFeature`.

## Build

```bash
cd /home/tang/autoware/vision_detection_ws
source /opt/ros/humble/setup.bash
source /home/tang/autoware/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

The recommended command starts both the USB camera and detection, and selects
the workspace-local TensorRT 8 runtime:

```bash
cd /home/tang/autoware/vision_detection_ws
./run_detection.sh /dev/video0
```

The default capture format is uncompressed `yuyv2rgb`. It avoids lower-frame
corruption observed with this camera's MJPEG stream at 640x480/30 FPS.

If an existing `usb_cam` node is already publishing `/image_raw`, run:

```bash
ros2 launch vision_detection_launch camera_detection.launch.xml \
  start_usb_camera:=false \
  image_topic:=/image_raw
```

YOLOX publishes known detections on `/vision_detection/rois`. The class-agnostic
node adds unknown obstacle proposals and publishes the merged result on
`/vision_detection/all_obstacles`, with the annotated image on
`/vision_detection/debug/image`. Green boxes are known YOLO objects; yellow boxes
are class-agnostic `UNKNOWN OBSTACLE` proposals. With a valid aligned depth sample,
each box is annotated as `DIST <range>m FWD <forward>m`; `DIST` is the 3D camera
distance and `FWD` is the forward distance. The overlay remains available even if
the optional camera-to-`base_link` TF is temporarily unavailable; in that case only
the 3D output is withheld.

If another camera driver is already running, disable the built-in USB driver and
pass its image topic:

```bash
ros2 launch vision_detection_launch camera_detection.launch.xml \
  start_usb_camera:=false \
  image_topic:=/your/camera/image_raw
```

If `/dev/video0` does not work, inspect available devices with `v4l2-ctl
--list-devices` or `ls -l /dev/video*`, then override `video_device`. Resolution,
frame rate, and pixel format are also launch arguments.

## Intel RealSense D455 distance-aware detection

The connected camera is an Intel RealSense D455. The standalone detector
subscribes to the D455 color image and raw depth image, and aligns raw depth to
color in software using `color/camera_info`, `depth/camera_info` and
`extrinsics/depth_to_color`.
For each 2D detection it estimates a robust depth inside the ROI, deprojects it
to 3D, and publishes the point in the configured target frame. The result is stored in
`DetectedObject.kinematics.pose_with_covariance.pose.position`; the debug image
also shows range (`DIST`) and forward distance (`FWD`). In standalone D455 mode the
target frame is `base_link`, so a valid camera-to-vehicle TF is required. In
addition to the ROI topic, the node publishes only detections with valid depth
and TF as standard `autoware_perception_msgs/msg/DetectedObjects` on
`/vision_detection/objects_3d`. The same message is also published by default
on `/perception/object_recognition/detection/camera_only/objects`, which is the
Autoware camera-only input channel for multi-object tracking.
The nearest valid forward distance is also published as a `std_msgs/msg/Float32`
on `/vision_detection/nearest_obstacle_distance` in meters; `-1.0` means no
valid depth was available for the current frame.

`run_camera_stream.sh` disables the camera-side `enable_sync`, `align_depth` and
post-processing filters by default (`CAMERA_DETECTION_DISABLE_ALIGN=true`).
This avoids the realsense-ros `asynchronous_syncer` stall that stops
`aligned_depth_to_color` after a short run. Set it to `false` only if another
consumer needs the camera-side aligned depth topic.

The detector also has an optional conservative round/ellipse-contour supplement.
It uses CLAHE, Canny contours, ellipse fitting, circularity, aspect-ratio,
pixel-size, and image-area limits; it does not run whole-image Hough detection
or merge candidates with the original unknown-obstacle boxes. Round/ellipse
candidates are added only when they do not overlap an existing YOLOX or contour
detection. The defaults are configured in `launch/camera_detection.launch.xml`;
disable the supplement with
`./run_realsense_detection.sh enable_round_contour:=false` if necessary.

Install the ROS driver once (this requires administrator privileges):

```bash
sudo apt-get update
sudo apt-get install --reinstall ros-humble-diagnostic-updater ros-humble-realsense2-camera
```

Then start the D455 driver and detection together:

```bash
source /opt/ros/humble/setup.bash
source /home/tang/autoware/install/setup.bash
source /home/tang/autoware/vision_detection_ws/install/setup.bash
ros2 launch vision_detection_launch realsense_detection.launch.xml
```

Or use the wrapper, which also adds the workspace-local TensorRT 8.5 library path
and hides stale optional Autoware overlay warnings:

```bash
./run_realsense_detection.sh
```

### Low-latency camera streaming

Video is streamed separately from rosbridge. Only one RealSense driver may
access the D455 at a time.

When Autoware is running, let Autoware own the camera. Optionally start the
custom detector without another RealSense driver:

```bash
cd /home/tang/autoware/vision_detection_ws
./run_autoware_camera_detection.sh
```

Then start the stream bridge in another terminal:

```bash
cd /home/tang/autoware/vision_detection_ws
./run_camera_stream.sh
```

The bridge automatically prefers Autoware's
`/sensing/camera/camera0/color/image_raw/compressed` topic. When Autoware is
not running, use `./run_realsense_detection.sh`; the bridge then falls back to
`/camera/color/image_raw/compressed`. It keeps only the newest JPEG frame,
encodes H.264 at a low-latency default of 600 kbps and 10 FPS, and publishes it to the local
MediaMTX server. Open either URL from a machine on the same ZeroTier network:

```text
WebRTC: http://10.245.188.140:8889/cam
HLS:    http://10.245.188.140:8888/cam
WHEP:   http://10.245.188.140:8889/cam/whep
```

WebRTC is the primary low-latency path. HLS is the fallback when UDP/ICE is
blocked. rosbridge remains responsible for controls, localization, and other
ROS messages, but should no longer carry camera frames.
The camera path is ZeroTier-only: WHEP signaling uses TCP 8889 and WebRTC
media uses UDP 8189 on `10.245.188.140`. The browser and vehicle computer must
therefore be joined to the same ZeroTier network.

Runtime overrides are available through environment variables:

```bash
CAMERA_STREAM_BITRATE=800k CAMERA_STREAM_BUFSIZE=800k CAMERA_STREAM_FPS=15 ./run_camera_stream.sh
CAMERA_STREAM_ENCODER=libx264 ./run_camera_stream.sh
CAMERA_STREAM_TOPIC=/standalone_camera_yolox/out/image/compressed ./run_camera_stream.sh
CAMERA_DETECTION_MODEL=/path/to/best.pt CAMERA_DETECTION_MODEL_SHA256=<sha256> ./run_camera_stream.sh
```

The default detector model is the integrated D455 three-class v3 weight at
`models/d455_v3_best.pt` (car, bollard, person). The stream wrapper verifies its
SHA-256 before loading it while retaining the raw-depth software alignment and
the existing `/d455_perception/detections` JSON output. Set
`CAMERA_DETECTION_MODEL_SHA256` to an empty string only when intentionally
testing a different model without a known checksum.

The camera driver publishes raw color and raw depth. The detector aligns raw
depth to color in software (camera_info + `depth_to_color` extrinsics) and
`run_camera_stream.sh` disables the camera-side sync/align filters by default.
Verify these topics before using the result for avoidance:

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/camera_info --once
ros2 topic echo /camera/extrinsics/depth_to_color --once
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 topic echo /vision_detection/all_obstacles --once
ros2 topic echo /vision_detection/objects_3d --once
ros2 topic echo /vision_detection/nearest_obstacle_distance --once
```

### Connect to Autoware object recognition

The camera node publishes `autoware_perception_msgs/msg/DetectedObjects` on
`/perception/object_recognition/detection/camera_only/objects`. The default
target frame is `base_link`; the camera-to-vehicle TF must exist, otherwise
detections with invalid transforms are discarded.

Build and run the camera detector:

```bash
cd /home/tang/autoware/vision_detection_ws
source /opt/ros/humble/setup.bash
source /home/tang/autoware/install/setup.bash
source install/setup.bash
./run_realsense_detection.sh
```

Verify the Autoware output:

```bash
ros2 topic type /perception/object_recognition/detection/camera_only/objects
ros2 topic echo /perception/object_recognition/detection/camera_only/objects --once
```

### Camera and radar fusion

The recommended path is to publish camera and radar detections separately and
let `autoware_multi_object_tracker` associate them. Use the camera topic above
as the `camera_streampetr` input channel and provide radar detections as the
`radar` input channel. Both messages must be
`autoware_perception_msgs/msg/DetectedObjects`, use `base_link`, and carry
close timestamps.

If a single fused detection topic is required, this workspace provides a
wrapper around Autoware's radar fusion node:

```bash
ros2 launch vision_detection_launch radar_fusion.launch.xml \
  input/radar_objects:=/perception/object_recognition/detection/radar/far_objects \
  output/objects:=/perception/object_recognition/detection/objects
```

The radar topic must already be converted and filtered to
`autoware_perception_msgs/msg/DetectedObjects`. Do not run this pre-tracking
fusion and the multi-channel camera/radar tracker fusion for the same inputs at
the same time, or objects will be duplicated.

If `base_link` is not connected to the D455 TF, the detector will keep the 2D
box but will not publish a fabricated 3D position. A static camera-to-vehicle
TF and a physical validation with targets at known distances are required
before using the output for braking or avoidance. Keep LiDAR/radar as an
independent safety source where available.

## Verify output

```bash
ros2 topic info /vision_detection/rois
ros2 topic hz /vision_detection/rois
ros2 topic echo /vision_detection/rois --once
```

View the annotated image:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/vision_detection/debug/image` in the topic selector.

## Compare YOLOX-Tiny and YOLOX-SPlus

Stop other camera/detection launches, then run:

```bash
cd /home/tang/autoware/vision_detection_ws
./run_compare.sh /dev/video0
```

Comparison topics:

```text
/vision_detection/yolox_tiny/rois
/vision_detection/yolox_tiny/image
/vision_detection/yolox_s/rois
/vision_detection/yolox_s/image
```

The default model is:

```text
/home/tang/autoware_data/ml_models/tensorrt_yolox/yolox-tiny.onnx
```

It is paired with the installed `yolox_tiny.param.yaml` configuration.

The first startup can take longer because TensorRT builds and caches an engine.
To select a different installed YOLOX model, override `model_path` on the launch
command line.
