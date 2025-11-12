from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    payload_mode_arg = DeclareLaunchArgument(
        "payload_mode",
        default_value="image",
        description="Chọn chế độ payload: 'video_stream' hoặc 'image'",
    )

    gimbal_node = Node(
        package="uav_mqtt_bridge",
        executable="gimbal",
        name="px4_mqtt_heartbeat",
        output="screen",
        parameters=[{"payload_mode": LaunchConfiguration("payload_mode")}],
    )

    uav_gps_node = Node(
        package="control_mqtt",
        executable="uav_gps_setpoint",
        name="uav_gps_setpoint",
        output="screen",
    )

    return LaunchDescription([payload_mode_arg, gimbal_node, uav_gps_node])
