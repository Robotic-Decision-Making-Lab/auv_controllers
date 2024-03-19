# Example 1: Individual Controller

This example uses the [integral sliding mode controller](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/tree/documentation/velocity_controllers) to demonstrate how
to launch a single controller.

## Tutorial Steps

1. To test the individual controller, run the following command:

    ```bash
    ros2 launch auv_control_demos individual.launch.py
    ```

  The launch file loads and starts the hardware interface and controller.

2. Check if the controller is running by the following commands:

    ```bash
    ros2 control list_controllers
    ```

    The output should resemble:

    ```bash
    integral_sliding_mode_controller[velocity_controllers/IntegralSlidingModeController] active
    ```

3. Verify if the hardware interfaces are loaded properly by opening another
  terminal and executing the following command:

    ```bash
    ros2 control list_hardware_interfaces
    ```

    The output should resemble:

    ```bash
    command interfaces
      integral_sliding_mode_controller/rx/velocity [available] [unclaimed]
      integral_sliding_mode_controller/ry/velocity [available] [unclaimed]
      integral_sliding_mode_controller/rz/velocity [available] [unclaimed]
      integral_sliding_mode_controller/x/velocity [available] [unclaimed]
      integral_sliding_mode_controller/y/velocity [available] [unclaimed]
      integral_sliding_mode_controller/z/velocity [available] [unclaimed]
      rx/effort [available] [claimed]
      ry/effort [available] [claimed]
      rz/effort [available] [claimed]
      x/effort [available] [claimed]
      y/effort [available] [claimed]
      z/effort [available] [claimed]
    state interfaces
      rx/velocity
      ry/velocity
      rz/velocity
      x/velocity
      y/velocity
      z/velocity
    ```

4. Open another terminal and execute the following command:

    ```bash
    ros2 topic pub /integral_sliding_mode_controller/system_state geometry_msgs/msg/Twist
    ```

5. In another terminal, execute the following:

    ```bash
    ros2 topic pub /integral_sliding_mode_controller/reference geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
    ```

6. If you echo the `/status' topic of any of the controllers, you should be able
  to see the controller status messages being populated. For example:

    ```bash
    ros2 topic echo /integral_sliding_mode_controller/status
    ```

    The output should resemble:

    ```bash
    ---
    header:
      stamp:
        sec: 1710817505
        nanosec: 673791332
      frame_id: ''
    dof_states:
    - name: x
      reference: 0.5
      feedback: 0.0
      feedback_dot: 0.0
      error: 0.5
      error_dot: 0.0
      time_step: 0.004459243
      output: 102.03798479512044
    - name: y
      reference: 0.0
      feedback: 0.0
      feedback_dot: 0.0
      error: 0.0
      error_dot: 0.0
      time_step: 0.004459243
      output: 0.0
    - name: z
      reference: 0.0
      feedback: 0.0
      feedback_dot: 0.0
      error: 0.0
      error_dot: 0.0
      time_step: 0.004459243
      output: -2.0
    - name: rx
      reference: 0.0
      feedback: 0.0
      feedback_dot: 0.0
      error: 0.0
      error_dot: 0.0
      time_step: 0.004459243
      output: 0.0
    - name: ry
      reference: 0.0
      feedback: 0.0
      feedback_dot: 0.0
      error: 0.0
      error_dot: 0.0
      time_step: 0.004459243
      output: 0.0
    - name: rz
      reference: 0.2
      feedback: 0.0
      feedback_dot: 0.0
      error: 0.2
      error_dot: 0.0
      time_step: 0.004459243
      output: 3.479864118728659
    ```

  This demonstrates that the integral sliding mode controller is functional, as
  the reference velocity commands sent to the controller are converted to the
  appropriate wrench values as reflected in the output field.

### Files used for this demo

- Launch Files:
  - [individual.launch.py](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/main/auv_control_demos/individual_controller/launch/individual.launch.py)

- Controllers:
  - [Integral Sliding Mode Controller](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/tree/documentation/velocity_controllers)

- Controller Config:
  - [individual_controller.yaml](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/main/auv_control_demos/individual_controller/config/individual_controller.yaml)

- Xacro Files:
  - [individual_config.xacro](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/main/auv_control_demos/individual_controllers/xacro/individual_config.xacro)
  - [individual_ros2_control.xacro](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/main/auv_control_demos/individual_controllers/xacro/individual_ros2_control.xacro)
  - [individual_urdf.xacro](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/main/auv_control_demos/individual_controllers/xacro/individual_urdf.xacro)
