# auv_controllers

auv_controllers is an open-source control platform for autonomous underwater
vehicles using ros2_control. This framework enables developers to implement and
deploy custom controllers to their underwater systems.

## Main Features

The main features of auv_controllers include:

- **Modular Control Design**: Utilizes `ros2_control` to easily integrate and
  deploy custom controllers for various AUV configurations and tasks.
- **Chainable Controllers**: Allows for the implementation of chainable or
  cascading controllers to handle complex control tasks.
- **Docker Integration**: Supports seamless deployment of algorithms to hardware
  using Docker containers.

## Installation

auv_controllers is currently supported on Linux and is available for the ROS2
Iron distribution. To install auv_controllers, first clone this project to the
`src` directory of your ROS workspace:

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



## Getting help

If you have questions regarding usage of auv_controllers or regarding
contributing to this project, please ask a question on our [Discussions](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/discussions) board!

## License

auv_controllers is released under the MIT license.
