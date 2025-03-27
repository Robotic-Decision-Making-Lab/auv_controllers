# Topic Sensors

This package provides a collection of Sensor interfaces that write incoming
ROS 2 messages to ros2_control state interfaces.

> [!NOTE]
> We could integrate subscribers into each of the controllers for the
> required state values. However, this introduces additional maintenance for
> controllers that require state feedback and increases complexity for
> multi-body systems whose states may come from multiple sources. Writing the
> ROS 2 messages directly to a state interface addresses these limitations at
> the cost of needing to launch a sensor.

## Odometry Sensor

A sensor plugin that reads incoming `nav_msgs/Odometry` messages and writes
them to state interfaces. No manipulation is done to the data, beyond proxying
the message values.

### Plugin Library

topic_sensors/odom_sensor

### Parameters

- topic: The topic that the sensor should read `nav_msgs/Odometry` messages on
- prefix: The state interface prefix. This is useful for scenarios, where the
  controller includes an additional prefix in the state interface.
