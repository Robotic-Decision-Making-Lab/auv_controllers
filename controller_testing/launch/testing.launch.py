from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("controller_testing"),
                        "urdf",
                        "testing.urdf.xacro",
                    ]
                ),
            ]
        ),
    }
    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[
                robot_description,
                PathJoinSubstitution(
                    [
                        FindPackageShare("controller_testing"),
                        "config",
                        "controllers.yaml",
                    ]
                ),
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "integral_sliding_mode_controller",
                "--controller-manager",
                ["", "controller_manager"],
            ],
        ),
    ]

    return LaunchDescription(nodes)
