# ros2_hik_camera

A ROS2 packge for Hikvision USB3.0 industrial camera

## Usage

```
colcon build
source install/setup.bash
ros2 launch hik_camera hik_camera.launch.py
```

ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.0183 image:=/image_raw camera:=/camera

## Params

- exposure_time
- gain
