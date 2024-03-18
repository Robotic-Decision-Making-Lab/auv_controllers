# AUV Control Demos

This package contains examples that demonstrate how to use the controllers in
`auv_controllers`. There are two examples. The first shows how to chain the
controllers together to a mock hardware interface for an AUV. The second example
shows how to lauch just a single controller on its own.

## Example 1: Controller Chaining

The example shows how to launch the auv controllers chained together and connect
them to a mock hardware interface. This framework takes a
velocity reference and state interfaces as input and converts them to their
appropriate PWM signal for the hardware interface.

The launch file [chaining.launch.py](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/feat-mock-hardware/auv_control_demos/launch/chaining.launch.py)
contains the code to launch all of the auv controllers chained to a mock
hardware interface, and that is the launch file we use for this first example.

### Tutorial Steps

1. To test the chainable controller framework, run the following command:

    ```bash
    ros2 launch auv_control_demos chaining.launch.py
    ```

    The launch file loads and starts the mock hardware and the controllers.

2. Check if controllers are running by:

    ```bash
    ros2 control list_controllers
    ```

    The output should resemble:

    ```bash
    integral_sliding_mode_controller[velocity_controllers/IntegralSlidingModeController] active
    thruster_allocation_matrix_controller[thruster_allocation_matrix_controller/ThrusterAllocationMatrixController] active
    thruster_1_controller[thruster_controllers/PolynomialThrustCurveController] active
    thruster_2_controller[thruster_controllers/PolynomialThrustCurveController] active
    thruster_3_controller[thruster_controllers/PolynomialThrustCurveController] active
    thruster_4_controller[thruster_controllers/PolynomialThrustCurveController] active
    thruster_5_controller[thruster_controllers/PolynomialThrustCurveController] active
    thruster_6_controller[thruster_controllers/PolynomialThrustCurveController] active
    thruster_7_controller[thruster_controllers/PolynomialThrustCurveController] active
    thruster_8_controller[thruster_controllers/PolynomialThrustCurveController] active
    ```

3. Verify if the hardware interfaces are loaded properly by opening another
  terminal and executing:

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
      thruster_1_controller/thruster_1_joint/effort [available] [claimed]
      thruster_1_joint/effort [available] [claimed]
      thruster_2_controller/thruster_2_joint/effort [available] [claimed]
      thruster_2_joint/effort [available] [claimed]
      thruster_3_controller/thruster_3_joint/effort [available] [claimed]
      thruster_3_joint/effort [available] [claimed]
      thruster_4_controller/thruster_4_joint/effort [available] [claimed]
      thruster_4_joint/effort [available] [claimed]
      thruster_5_controller/thruster_5_joint/effort [available] [claimed]
      thruster_5_joint/effort [available] [claimed]
      thruster_6_controller/thruster_6_joint/effort [available] [claimed]
      thruster_6_joint/effort [available] [claimed]
      thruster_7_controller/thruster_7_joint/effort [available] [claimed]
      thruster_7_joint/effort [available] [claimed]
      thruster_8_controller/thruster_8_joint/effort [available] [claimed]
      thruster_8_joint/effort [available] [claimed]
      thruster_allocation_matrix_controller/rx/effort [available] [claimed]
      thruster_allocation_matrix_controller/ry/effort [available] [claimed]
      thruster_allocation_matrix_controller/rz/effort [available] [claimed]
      thruster_allocation_matrix_controller/x/effort [available] [claimed]
      thruster_allocation_matrix_controller/y/effort [available] [claimed]
      thruster_allocation_matrix_controller/z/effort [available] [claimed]
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
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/integral_sliding_mode_controller/reference
    ```

6. If you echo the `/status' topic of any of the controllers, you would be able
  to see the controller status messages being populated. For example:

    ```bash
    ros2 topic echo /thruster_2_controller/status
    ```

    The output should resemble:

    ```bash
    ---
    header:
      stamp:
        sec: 1710794577
        nanosec: 621312555
      frame_id: ''
    dof_state:
      name: thruster_2_joint
      reference: -28.30267593385744
      feedback: 0.0
      feedback_dot: 0.0
      error: 0.0
      error_dot: 0.0
      time_step: 0.004444319
      output: 1182.0
    ```

  This clearly demonstrates that the controller chaining is functional, as the
  reference velocity commands sent to the integral_sliding_mode_controller are
  converted to the appropriate PWM values as part of the chained controllers and
  are reflected in the output of the thruster_controller.

### Files used for this demo

- Launch Files:
  - [chaining.launch.py](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/feat-mock-hardware/auv_control_demos/launch/chaining.launch.py)

- Controllers:
  - [Integral Sliding Mode Controller](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/tree/documentation/velocity_controllers)
  - [Thruster Allocation Matrix Controller](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/tree/documentation/thruster_allocation_matrix_controller)
  - [Polynomial Thrust Curve Controller](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/tree/documentation/thruster_controllers)

- Controller Config:
  - [controllers.yaml](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/feat-mock-hardware/auv_control_demos/config/controllers.yaml)

- URDF File:
  - [testing.urdf.xacro](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/feat-mock-hardware/auv_control_demos/urdf/testing.urdf.xacro)

## Example 2: Individual Controller
