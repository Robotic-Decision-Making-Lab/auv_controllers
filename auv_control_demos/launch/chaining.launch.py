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
                    "chaining.urdf.xacro",
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

    thruster_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                f"thruster_{i + 1}_controller",
                "--controller-manager",
                ["", "controller_manager"],
            ],
        )
        for i in range(8)
    ]

    delay_thruster_spawners = []
    for i, thruster_spawner in enumerate(thruster_spawners):
        if not len(delay_thruster_spawners):
            delay_thruster_spawners.append(
                thruster_spawner,
            )
        else:
            delay_thruster_spawners.append(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=thruster_spawners[i - 1],
                        on_exit=[thruster_spawner],
                    )
                )
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
    delay_tam_controller_spawner_after_thruster_controller_spawners = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=thruster_spawners[-1],
                on_exit=[tam_controller_spawner],
            )
        )
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
                        FindPackageShare("auv_control_demos"),
                        "config",
                        "chained_controllers.yaml",
                    ]
                ),
            ],
        ),
        *delay_thruster_spawners,
        delay_tam_controller_spawner_after_thruster_controller_spawners,
        delay_velocity_controller_spawner_after_tam_controller_spawner,
    ]

    return LaunchDescription(nodes)
