"""Live sensor plus mounting TF and RViz. Angles are degrees, lengths metres."""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def number(name):
        result = float(value(name))
        if not math.isfinite(result):
            raise ValueError(f'{name} must be finite')
        return result

    def enabled(name):
        return value(name).lower() in ('true', '1', 'yes')

    parent, child = value('parent_frame'), value('sensor_frame')
    if not parent or not child or parent == child:
        raise ValueError('parent_frame and sensor_frame must be distinct nonempty frames')
    xyz = [number(axis) for axis in ('x', 'y', 'z')]
    rpy = [math.radians(number(axis)) for axis in ('roll_deg', 'pitch_deg', 'yaw_deg')]
    actions = []
    if not enabled('mounting_verified'):
        actions.append(LogInfo(msg='PROVISIONAL mounting: verify yaw, axis signs and origin. '
                               'Wrench values remain in the native sensor frame.'))
    actions.append(Node(
        package='kunwei_ft_driver', executable='driver', output='screen',
        parameters=[{'host': value('host'), 'port': int(value('port')),
                     'frame_id': child, 'unit_scale': number('unit_scale')}],
        remappings=[('ft_sensor/wrench', value('wrench_topic'))],
    ))
    if enabled('publish_tf'):
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='kunwei_mount_tf', output='screen', arguments=[
                '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                '--frame-id', parent, '--child-frame-id', child],
        ))
    if enabled('rviz'):
        config = os.path.join(get_package_share_directory('kunwei_ft_driver'),
                              'rviz', 'forces.rviz')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='kunwei_rviz', output='screen',
            arguments=['-d', config, '-f', value('fixed_frame') or parent],
            remappings=[('/ft_sensor/wrench', value('wrench_topic'))],
        ))
    return actions


def generate_launch_description():
    arguments = [
        ('host', '192.168.1.88', 'Sensor TCP address'),
        ('port', '5152', 'Sensor TCP port'),
        ('parent_frame', 'fr3_link8', 'Existing robot flange frame'),
        ('sensor_frame', 'ft_sensor', 'Native measurement frame'),
        ('fixed_frame', '', 'RViz fixed frame; empty uses parent_frame'),
        ('wrench_topic', '/ft_sensor/wrench', 'WrenchStamped output'),
        ('x', '0.0', 'Sensor origin X in parent, metres (unmeasured placeholder)'),
        ('y', '0.0', 'Sensor origin Y in parent, metres (unmeasured placeholder)'),
        ('z', '0.0', 'Sensor origin Z in parent, metres (measure adapter stack)'),
        ('roll_deg', '0.0', 'Nominal mounting roll supported by three-axis force comparison'),
        ('pitch_deg', '0.0', 'Mount pitch, degrees'),
        ('yaw_deg', '45.0', 'Nominal sensor +X heading in parent XY, supported by force comparison'),
        ('unit_scale', '9.80665', 'Raw kgf/kgf*m to N/N*m'),
        ('publish_tf', 'true', 'False if your robot URDF already defines the sensor TF'),
        ('mounting_verified', 'false', 'Suppress provisional mounting reminder'),
        ('rviz', 'true', 'Start RViz window'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in arguments
    ] + [OpaqueFunction(function=launch_nodes)])
