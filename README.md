mc_rtc_ros_control
==

Integration package between mc_rtc and ROS2 control. Provides a node subscribe to a robot's JointState and publishing a message suitable for a position_controllers/JointGroupPositionController controller.

**ROS1 implementation can be found [here](https://github.com/mc-rtc/mc_rtc_ros_control/tree/noetic).**


Requirements
--

Build as a catkin package. This has the following required dependencies:

- rclcpp
- std_msgs
- sensor_msgs
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)

Usage
--

```bash
ros2 launch mc_rtc_ros_control control.launch.py publish_to:=/my/command subscribe_to:=/my/state
```

Where:

- `publish_to` is the topic where the controller is subscribed to a control message (defaults to: `/command`)
- `subscribe_to` is the topic where the robot is publishing its state through a `sensor_msgs/msg/JointState` message (defaults to: `/joint_state`)

Visualization
--

### Convex visualization

Display the collision convex shapes of a robot in RViz2:

```bash
ros2 launch mc_rtc_ros_control convex_visualization.launch.py
ros2 launch mc_rtc_ros_control convex_visualization.launch.py robot:=JVRC1 frame_id:=world
```

Where:

- `robot` is the mc_rtc robot module name (defaults to: `JVRC1`)
- `frame_id` is the TF reference frame for the markers (defaults to: `map`)

### Surface visualization

Display the contact surfaces of a robot alongside its mesh in RViz2:

```bash
ros2 launch mc_rtc_ros_control surface_visualization.launch.py
ros2 launch mc_rtc_ros_control surface_visualization.launch.py robot:=JVRC1 frame_id:=world
```

Parameters are the same as above. This node also publishes the robot URDF on `/robot_description` and static TF for all bodies so that the robot model is rendered in RViz2.

Example
--

This example focus on UR usage :

Requirements :

* https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation

Run the following command to start the simulation :

```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py initial_joint_controller:=forward_position_controller
```

To run mc_rtc ros node, please run :

```bash
ros2 launch mc_rtc_ros_control control.launch.py publish_to:=/forward_position_controller/commands
```
