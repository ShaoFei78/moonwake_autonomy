from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("moonwake_ros2_training"),
        "config",
        "training_params.yaml",
    ])

    return LaunchDescription([
        Node(
            package="moonwake_ros2_training",
            executable="target_offset_publisher",
            name="target_offset_publisher",
            output="screen",
            parameters=[config],
            remappings=[
                ("target_offset", "/training/target_offset"),
            ],
        ),
        Node(
            package="moonwake_ros2_training",
            executable="steering_hint_node",
            name="steering_hint_node",
            output="screen",
            parameters=[config],
            remappings=[
                ("target_offset", "/training/target_offset"),
                ("steering_hint", "/training/steering_hint"),
            ],
        ),
    ])