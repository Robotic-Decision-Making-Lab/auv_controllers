# auv_controllers

auv_controllers is a collection of chainable controllers for autonomous
underwater vehicles (AUVs) implemented using ros2_control. The controllers have
been designed to support the complete AUV control hierarchy and to enable
benchmarking against other commonly-used control algorithms.

> [!NOTE]
> If you are interested in adding your own controller to this project, please
> consider submitting a [pull request](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/pulls)!

## Installation

auv_controllers is currently supported on Linux and is available for the ROS 2
Rolling, Jazzy, Iron and Humble distributions. To install auv_controllers,
first clone this project to the `src` directory of your ROS workspace:

```bash
git clone git@github.com:Robotic-Decision-Making-Lab/auv_controllers.git
```

After cloning the project, install all external dependencies using `vcs`:

```bash
vcs import src < src/auv_controllers/ros2.repos
```

Finally, install the ROS dependencies using `rosdep`:

```bash
rosdep update && \
rosdep install -y --from-paths src --ignore-src
```

## Quick start

To learn more about how to use the controllers provided in this project, please
refer to the [examples package](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/tree/main/auv_control_demos).

## Getting help

If you have questions regarding usage of auv_controllers or regarding
contributing to this project, please ask a question on our [Discussions](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/discussions) board!

## License

auv_controllers is released under the MIT license.
