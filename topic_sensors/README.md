# Topic Sensors

This package provides a collection of sensor interfaces that write incoming
ROS 2 messages to ros2_control state interfaces.

## Odometry Sensor

A sensor plugin that reads incoming `nav_msgs/Odometry` messages and writes
them to state interfaces.

### Plugin Library

topic_sensors/odom_sensor

### Parameters

- prefix: The node prefix.
- topic: The topic that the sensor should subscribe to.
- transform_message: Whether or not the sensor should transform the message
  from the ROS [REP-105](https://ros.org/reps/rep-0105.html) convention to the
  maritime convention documented in [REP-156](https://github.com/ros-infrastructure/rep/pull/398).
