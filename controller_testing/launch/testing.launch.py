from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "integral_sliding_mode_controller",
            "--controller-manager",
            ["", "controller_manager"],
        ],
    )
    tam_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "thruster_allocation_matrix_controller",
            "--controller-manager",
            ["", "controller_manager"],
        ],
    )
    delay_velocity_controller_spawner_after_tam_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=tam_controller_spawner,
                on_exit=[velocity_controller_spawner],
            )
        )
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
                        FindPackageShare("controller_testing"),
                        "config",
                        "controllers.yaml",
                    ]
                ),
            ],
        ),
        tam_controller_spawner,
        velocity_controller_spawner,
    ]

    return LaunchDescription(nodes)
