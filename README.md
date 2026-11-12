# Kunwei FT driver

ROS 2 driver and C++ acquisition library for the Kunwei Ethernet force/torque sensor.
Publishes `geometry_msgs/msg/WrenchStamped` on `/ft_sensor/wrench`, with an optional 30 Hz display stream on `/ft_sensor/wrench_visualization`.

## Install

Place this package in `~/local_ws/src/`, then build from the workspace root:

```bash
source /opt/ros/humble/setup.bash
cd ~/local_ws
rosdep install --from-paths src/kunwei_ft_driver --ignore-src -r -y
colcon build --packages-select kunwei_ft_driver --symlink-install
source install/setup.bash
```

## Run

Connect the sensor to the network and use its address:

```bash
ros2 launch kunwei_ft_driver visualize.launch.py host:=192.168.1.88 port:=5152
ros2 topic echo /ft_sensor/wrench
```

The visualization shows a KWR75B sensor with the 6 mm ISO 9409-1-50-4 adapter.
Preview the geometry without connecting to hardware:

```bash
ros2 launch kunwei_ft_driver visualize.launch.py start_driver:=false
```

For acquisition without the visualization:

```bash
ros2 launch kunwei_ft_driver publish.launch.py host:=192.168.1.88
```

This starts only the sensor publisher, with no RViz or TF publisher.
The lower-frequency visualization stream is disabled by default, including when running the executable directly.
Enable it with `publish_visualization:=true` (or `-p publish_visualization:=true` when running the executable directly).
`visualize.launch.py` enables it automatically.

The publisher launch also accepts `host`, `port`, `sensor_prefix`, `sensor_frame`, `unit_scale`, `visualization_rate`, `wrench_topic` and `visualization_topic`.

Set `sensor_prefix` to change the ROS interfaces and frame names (default: `ft_sensor`):

```bash
ros2 launch kunwei_ft_driver visualize.launch.py sensor_prefix:=wrist_ft
ros2 launch kunwei_ft_driver publish.launch.py sensor_prefix:=wrist_ft
ros2 run kunwei_ft_driver kunwei_ft_publisher --ros-args -p sensor_prefix:=wrist_ft
```

This uses `/wrist_ft/wrench`, `/wrist_ft/zero` and frame `wrist_ft`, plus `/wrist_ft/wrench_visualization` when enabled.
Prefixes are relative ROS names without leading or trailing slashes; nested prefixes such as `robot1/ft_sensor` are supported.
Use `frame_id` on the executable or `sensor_frame` on the launch to override the measurement frame independently.
Use `wrench_topic` and `visualization_topic` to override the launch's output topics independently.

The default `unit_scale` is `9.80665` (kgf/kgf·m to N/N·m).
Set it to `1.0` if sensor output is already in SI units.
Software zero is available after receiving samples: `ros2 service call /ft_sensor/zero std_srvs/srv/Trigger '{}'`.

## Embed in a robot description

Include the component inside a robot Xacro and attach it to an existing link:

```xml
<xacro:include filename="$(find kunwei_ft_driver)/urdf/kunwei_sensor.urdf.xacro"/>
<xacro:kunwei_sensor parent="tool_flange" sensor_frame="ft_sensor"/>
```

The macro adds the adapter, sensor body and native measurement frame.
It accepts `xyz`, `roll`, `pitch`, `yaw`, `adapter_yaw`, `native_frame_yaw`, `adapter_height` and `sensor_height` (metres and radians).
The default native yaw is 120°; adapter yaw is 0°.
Use the measurement frame as the publisher's `frame_id`.
