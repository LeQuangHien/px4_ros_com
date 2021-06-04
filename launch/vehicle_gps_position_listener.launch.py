from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='px4_ros_com', node_executable='vehicle_gps_position_listener', output='screen'),
    ])