"""KWR75B with the factory adapter"""
import math
import os
import re
import tempfile
from pathlib import Path

import yaml
import xacro
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node


def mounting_description(parent, child, xyz, rpy, adapter_height, sensor_height,
                         native_yaw_offset=math.pi, adapter_yaw=0.0):
    """Expand the same component used by robot descriptions."""
    path = os.path.join(get_package_share_directory('kunwei_ft_driver'),
                        'urdf', 'kunwei_sensor.urdf.xacro')
    source = f'''<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="kunwei_mount">
      <xacro:include filename="{path}"/>
      <link name="{parent}"/>
      <xacro:kunwei_sensor parent="{parent}" sensor_frame="{child}"
          xyz="{' '.join(map(str, xyz))}" roll="{rpy[0]}" pitch="{rpy[1]}"
          yaw="{rpy[2]}" adapter_yaw="{adapter_yaw}"
          native_frame_yaw="{native_yaw_offset}" adapter_height="{adapter_height}"
          sensor_height="{sensor_height}"/>
    </robot>'''
    # URDF names cannot contain whitespace or XML delimiters.
    for frame in (parent, child):
        if any(ch.isspace() or ch in '<>&\"\'' for ch in frame):
            raise ValueError('frame names must not contain whitespace or XML delimiters')
    document = xacro.parse(source)
    xacro.process_doc(document)
    return document.toxml()


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

    sensor_prefix = value('sensor_prefix')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', sensor_prefix):
        raise ValueError('sensor_prefix must be a nonempty relative ROS name without leading/trailing slashes')
    parent, child = value('parent_frame'), value('sensor_frame')
    if not parent or not child or parent == child:
        raise ValueError('parent_frame and sensor_frame must be distinct nonempty frames')
    xyz = [number(axis) for axis in ('x', 'y', 'z')]
    rpy = [math.radians(number(axis)) for axis in ('roll_deg', 'pitch_deg', 'yaw_deg')]
    variant = value('sensor_model').upper()
    if variant not in ('A', 'B', 'C', 'D', 'E', 'F'):
        raise ValueError('sensor_model must be A, B, C, D, E or F')
    sensor_height = 0.0335 if variant in ('C', 'D') else 0.0315
    adapter_height = number('adapter_height')
    if adapter_height <= 0:
        raise ValueError('adapter_height must be positive')
    actions = []
    if not enabled('mounting_verified'):
        actions.append(LogInfo(msg=(
            f'Kunwei adapter and sensor: native sensor yaw={value("yaw_deg")} deg, '
            f'adapter={adapter_height} m, sensor={sensor_height} m. '
            'The sensor body and native wrench axes are separate TF frames.')))
    if enabled('start_driver'):
        actions.append(Node(
            package='kunwei_ft_driver', executable='kunwei_ft_publisher', output='screen',
            parameters=[{'host': value('host'), 'port': int(value('port')),
                         'sensor_prefix': sensor_prefix, 'frame_id': child, 'unit_scale': number('unit_scale'),
                         'publish_visualization': True,
                         'visualization_rate': number('visualization_rate')}],
            remappings=[(sensor_prefix + '/wrench', value('wrench_topic')),
                        (sensor_prefix + '/wrench_visualization', value('visualization_topic'))],
        ))
    if enabled('publish_tf'):
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace='kunwei_mount', name='state_publisher', output='screen',
            parameters=[{'robot_description': ParameterValue(mounting_description(
                parent, child, xyz, rpy, adapter_height, sensor_height,
                math.radians(number('native_frame_yaw_deg')),
                math.radians(number('adapter_yaw_deg'))), value_type=str)}],
        ))
    if enabled('rviz'):
        config = os.path.join(get_package_share_directory('kunwei_ft_driver'),
                              'rviz', 'forces.rviz')
        with open(config, encoding='utf-8') as stream:
            rviz_config = yaml.safe_load(stream)
        for display in rviz_config['Visualization Manager']['Displays']:
            if display['Class'] == 'rviz_default_plugins/Axes':
                frame = display['Reference Frame']
                if frame.startswith('ft_sensor'):
                    display['Reference Frame'] = child + frame[len('ft_sensor'):]
        runtime = tempfile.TemporaryDirectory(prefix='kunwei-rviz-')
        config = str(Path(runtime.name) / 'forces.rviz')
        Path(config).write_text(yaml.safe_dump(rviz_config, sort_keys=False))

        def cleanup(_context):
            runtime.cleanup()
            return []

        actions.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=cleanup)])))
        actions.append(Node(
            package='rviz2', executable='rviz2', name='kunwei_rviz', output='screen',
            arguments=['-d', config, '-f', value('fixed_frame')],
            remappings=[('/ft_sensor/wrench_visualization', value('visualization_topic'))],
        ))
    return actions


def generate_launch_description():
    arguments = [
        ('host', '192.168.1.88', 'Sensor TCP address'),
        ('port', '5152', 'Sensor TCP port'),
        ('parent_frame', 'base_link', 'Existing robot flange frame'),
        ('sensor_prefix', 'ft_sensor', 'Relative ROS prefix for topics, zero service and default sensor frame'),
        ('sensor_frame', LaunchConfiguration('sensor_prefix'), 'Native measurement frame'),
        ('fixed_frame', 'base_link', 'RViz fixed frame'),
        ('wrench_topic', ['/', LaunchConfiguration('sensor_prefix'), '/wrench'], 'WrenchStamped output'),
        ('visualization_topic', ['/', LaunchConfiguration('sensor_prefix'), '/wrench_visualization'], 'Latest wrench for RViz'),
        ('visualization_rate', '30.0', 'RViz wrench update rate in Hz; full-rate output unchanged'),
        ('x', '0.0', 'Adapter contact X in parent, metres'),
        ('y', '0.0', 'Adapter contact Y in parent, metres'),
        ('z', '0.0', 'Extra adapter offset Z before the factory adapter, metres'),
        ('roll_deg', '0.0', 'Sensor mounting roll in degrees'),
        ('pitch_deg', '0.0', 'Mount pitch, degrees'),
        ('yaw_deg', '120.0', 'Native wrench +X heading in the parent frame'),
        ('adapter_yaw_deg', '0.0', 'Factory adapter bolt-pattern heading in parent XY'),
        ('native_frame_yaw_deg', '180.0', 'Native XY rotation from the sensor body frame'),
        ('unit_scale', '9.80665', 'Raw kgf/kgf*m to N/N*m'),
        ('sensor_model', 'B', 'KWR75 variant A/B/E/F: 31.5 mm; C/D: 33.5 mm'),
        ('adapter_height', '0.006', 'Factory adapter contact-plane thickness, metres'),
        ('start_driver', 'true', 'Connect to the FT sensor; false for geometry preview'),
        ('publish_tf', 'true', 'Publish mounting TF and model; false if defined in robot URDF'),
        ('mounting_verified', 'false', 'Suppress mounting summary'),
        ('rviz', 'true', 'Start RViz window'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in arguments
    ] + [OpaqueFunction(function=launch_nodes)])
