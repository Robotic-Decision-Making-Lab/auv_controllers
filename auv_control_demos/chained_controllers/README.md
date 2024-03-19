# Example 2: Controller Chaining

This example demonstrates how to launch multiple controllers chained together.
The control hierarchy uses a velocity reference input with state feedback and
applies a PWM command to the hardware interface.

## Tutorial Steps

1. To launch the chainable control framework, run the following command:

   ```bash
   ros2 launch auv_control_demos chaining.launch.py
   ```

   This launch file loads and starts the hardware interface and controllers.

2. Execute the following command to verify that the controllers are running:

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

3. Verify that the hardware interfaces have been properly loaded by opening
   another terminal and executing:

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

4. State feedback can be provided to ISMC using the controller's state
   interfaces or a topic. To demonstrate the topic-based interface, run the
   following command:

   ```bash
   ros2 topic pub /integral_sliding_mode_controller/system_state geometry_msgs/msg/Twist
   ```

5. The ISMC accepts reference commands sent over a topic or a reference
   interface. Run the following command in a separate terminal to provide the
   controller with a dummy reference input:

   ```bash
   ros2 topic pub /integral_sliding_mode_controller/reference geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
   ```

6. Each of the implemented controllers publishes controller state information to
   their respective `~/status` topics. For example,

   ```bash
   ros2 topic echo /thruster_2_controller/status
   ```

   Should yield an output similar to the following:

   ```bash
   header:
     stamp:
       sec: 1710818346
       nanosec: 252453476
     frame_id: ''
   dof_state:
     name: thruster_2_joint
     reference: -39.79632414479818
     feedback: 0.0
     feedback_dot: 0.0
     error: 0.0
     error_dot: 0.0
     time_step: 0.033380572
     output: 1134.0
   ```

   This output indicates that the controller chaining was successful. In
   particular, we can see that the velocity commands sent to the
   `integral_sliding_mode_controller` are converted to thrust values, which is
   reflected in the `thruster_controller` output.

## Files used for this demo

- Launch Files:
  - [chaining.launch.py](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/main/auv_control_demos/chained_controllers/launch/chaining.launch.py)

- Controllers:
  - [Integral Sliding Mode Controller](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/tree/main/velocity_controllers)
  - [Thruster Allocation Matrix Controller](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/tree/main/thruster_allocation_matrix_controller)
  - [Polynomial Thrust Curve Controller](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/tree/main/thruster_controllers)

- Controller Config:
  - [chained_controllers.yaml](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/main/auv_control_demos/chained_controllers/config/chained_controllers.yaml)

- Xacro Files:
  - [chained_config.xacro](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/main/auv_control_demos/chained_controllers/xacro/chained_config.xacro)
  - [chained_ros2_control.xacro](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/main/auv_control_demos/chained_controllers/xacro/chained_ros2_control.xacro)
  - [chained_urdf.xacro](https://github.com/Robotic-Decision-Making-Lab/auv_controllers/blob/main/auv_control_demos/chained_controllers/xacro/chained_urdf.xacro)
