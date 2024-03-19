from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("auv_control_demos"),
                    "urdf",
                    "individual.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "integral_sliding_mode_controller",
            "--controller-manager",
            ["", "controller_manager"],
        ],
    )

    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[
                robot_description,
                PathJoinSubstitution(
                    [
                        FindPackageShare("auv_control_demos"),
                        "config",
                        "individual_controller.yaml",
                    ]
                ),
            ],
        ),
        velocity_controller_spawner,
    ]

    return LaunchDescription(nodes)
