from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("uav_mqtt_bridge")]
    )


    gimbal_node = Node(
        package="uav_mqtt_bridge",
        executable="gimbal",
        name="px4_mqtt_heartbeat",
        output="screen",
    )

    uav_gps_node = Node(
        package="control_mqtt",
        executable="uav_gps_setpoint",
        name="uav_gps_setpoint",
        output="screen",
    )

    return LaunchDescription([gimbal_node, uav_gps_node])
