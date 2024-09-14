# Copyright 2024, Everardo Gonzalez
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description to run the individual controller demo.

    Returns:
        The example launch description.
    """
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("auv_control_demos"),
                    "xacro",
                    "individual_config.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="both",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("auv_control_demos"),
                            "config",
                            "individual_controller.yaml",
                        ]
                    ),
                ],
                remappings=[
                    ("/controller_manager/robot_description", "/robot_description"),
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
    )
