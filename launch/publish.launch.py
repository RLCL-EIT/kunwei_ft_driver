"""Publish Kunwei force/torque data without starting RViz or a TF publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    prefix = LaunchConfiguration('sensor_prefix')
    arguments = [
        ('host', '192.168.1.88', 'Sensor TCP address'),
        ('port', '5152', 'Sensor TCP port'),
        ('sensor_prefix', 'ft_sensor', 'Relative ROS prefix for topics and zero service'),
        ('sensor_frame', prefix, 'Native measurement frame'),
        ('unit_scale', '9.80665', 'Raw kgf/kgf*m to N/N*m'),
        ('publish_visualization', 'false', 'Enable the additional display stream'),
        ('visualization_rate', '30.0', 'Display stream rate in Hz'),
        ('wrench_topic', ['/', prefix, '/wrench'], 'WrenchStamped output'),
        ('visualization_topic', ['/', prefix, '/wrench_visualization'], 'Latest wrench for display'),
    ]
    parameters = {
        'host': ParameterValue(LaunchConfiguration('host'), value_type=str),
        'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
        'sensor_prefix': ParameterValue(prefix, value_type=str),
        'frame_id': ParameterValue(LaunchConfiguration('sensor_frame'), value_type=str),
        'unit_scale': ParameterValue(LaunchConfiguration('unit_scale'), value_type=float),
        'publish_visualization': ParameterValue(
            LaunchConfiguration('publish_visualization'), value_type=bool),
        'visualization_rate': ParameterValue(
            LaunchConfiguration('visualization_rate'), value_type=float),
    }
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in arguments
    ] + [
        Node(
            package='kunwei_ft_driver', executable='kunwei_ft_publisher', output='screen',
            parameters=[parameters],
            remappings=[
                ([prefix, '/wrench'], LaunchConfiguration('wrench_topic')),
                ([prefix, '/wrench_visualization'], LaunchConfiguration('visualization_topic')),
            ],
        ),
    ])
