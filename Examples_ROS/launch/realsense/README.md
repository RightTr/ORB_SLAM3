# RealSense D435i (serial 939622073378)

These launch files start only ORB-SLAM3 and, where enabled, RViz. Start the
RealSense ROS 2 camera driver separately from `~/realsense_ws`.

All settings use the connected device's 640x480 calibration and 30 Hz camera
profiles. If the camera or image profile changes, update the corresponding
configuration in `config/realsense` from that device's camera info and
extrinsics.

For color mono, RGB-D, or RGB-D inertial, start the camera with 640x480 color
and depth. RGB-D also requires depth aligned to color:

```bash
source ~/realsense_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30 \
  align_depth.enable:=true enable_sync:=true
```

For RGB-D inertial, additionally pass
`enable_gyro:=true enable_accel:=true unite_imu_method:=2`.

For stereo or stereo inertial, enable both infrared streams at 640x480:

```bash
source ~/realsense_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_infra1:=true enable_infra2:=true \
  depth_module.infra_profile:=640x480x30 enable_sync:=true
```

For stereo inertial, additionally pass
`enable_gyro:=true enable_accel:=true unite_imu_method:=2`.

Then run one of:

```bash
source ~/orbslam3_ws/install/setup.bash
ros2 launch orb_slam3 mono_realsense_d435i.launch.py
ros2 launch orb_slam3 rgbd_realsense_d435i.launch.py
ros2 launch orb_slam3 rgbd_inertial_realsense_d435i.launch.py
ros2 launch orb_slam3 stereo_realsense_d435i.launch.py
ros2 launch orb_slam3 stereo_inertial_realsense_d435i.launch.py
```

The topic names come from each mode's YAML file. Override a topic at launch
time with, for example, `rgb_topic:=/other/color/image_raw`. Pangolin is
enabled by default; use `start_rviz:=false` to avoid opening RViz as well.
