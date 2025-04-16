# Topic Sensors

This package provides a collection of Sensor interfaces that write incoming
ROS 2 messages to ros2_control state interfaces.

## Odometry Sensor

A sensor plugin that reads incoming `nav_msgs/Odometry` messages and writes
them to state interfaces. No manipulation is done to the data, beyond proxying
the message values.

### Plugin Library

topic_sensors/odom_sensor
