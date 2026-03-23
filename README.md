# MADAR Manipulation Using Visual Guidance Data

This repository contains the guidelines and some data from the project related to this work. The following README explains the main parts of the project core:

## Índex

1. [Robot Arm Control Framework](#1-robot-arm-control-framework)  
2. [MADAR Synchronization and Coordination Framework](#2-madar-Synchronization-and-coordination-framework)  
3. [OAK-D Head](#3-oak-d-head)

The `testing_inputs` folder includes safety operation paths for testing the implemented pipeline. More data is available in the official repositories.

# 1. Robot Arm Control Framework

Provides a modular and extensible C++ framework for controlling a 6-degree-of-freedom robot arm. It integrates with ROS 2 and offers multiple control strategies, including PID-based joint velocity control, Jacobian-based Cartesian velocity control, and direct velocity bypass control. The system encompasses kinematics computations, control loops, and various ROS 2-compatible interfaces to enable flexible, real-time manipulation.

---

## Conceptual Architecture

The architecture centers around the `RobotArm` class, which inherits from `RobotArmKinematics` and integrates all aspects of the arm’s state, control logic, and communication interfaces. The main components include:

- **RobotArmKinematics**  
  Responsible for forward and inverse kinematics computations using Eigen and a dedicated IK solver (`KinenikUR`). It manages coordinate transformations, pose computations, and trajectory generation in Cartesian and joint spaces.

- **RobotArm Class**  
  Acts as the central manager for the robot arm, maintaining the current and desired joint states, initializing and switching between controllers, running the control loop in a dedicated thread, and interfacing with ROS 2 communication layers to send and receive commands and feedback.


### Kinematic Offsets

The `robot_arm` framework allows defining **base** and **TCP** offsets within the kinematics module. This means you can specify transforms that shift the coordinate frames away from the robot’s hardware default frames to more useful or task-relevant reference frames.

For example, you might define a TCP offset to align the control frame to the **palm of the hand** or the **tip of a gripper**, rather than the default wrist 3 link. Similarly, a base offset can shift the reference frame from the robot base to a custom workspace frame.

These offsets are applied internally during forward and inverse kinematics computations, enabling the system to:

- Work naturally with frames that better represent the real tool or task geometry.
- Perform motion planning and control relative to custom coordinate frames without modifying the physical robot model.
- Increase flexibility and abstraction in applications involving end-effectors or tools attached to the robot.

Offsets are configured via the `setBaseTransformationOffset()` and `setTCPTransformationOffset()` methods of the `RobotArmKinematics` class and are used transparently in trajectory and control computations.

This design enhances modularity and prepares the framework to support different robot setups and tools beyond the underlying hardware.


---

## Controllers

Control logic is encapsulated in multiple controller implementations, all derived from the abstract `RobotArmController` base class that defines the required interface for initialization and velocity command computation. The system currently includes:

- **PIDController**  
  A joint-level PID velocity controller that dynamically loads gains from YAML configuration files. It computes commands to correct position and velocity errors for precise tracking.

- **JacobianController**  
  A Cartesian velocity controller leveraging geometric and analytic Jacobians. It transforms TCP pose errors into joint velocity commands to guide the end-effector along desired Cartesian trajectories.

- **BypassVelocityController**  
  A passthrough controller that forwards externally provided joint velocity commands without modification. Useful for manual or external direct velocity control scenarios.

The modular design allows developers to implement new controllers by subclassing `RobotArmController` and registering them within the controller factory for seamless integration.

---

## Control Interfaces

The framework exposes several ROS 2 control interfaces tailored to different application needs:

- **Follow Joint Trajectory Interface**  
  Uses the `PIDController` to execute joint trajectories received as ROS 2 action goals (`FollowJointTrajectory`), ensuring smooth and timely motion.

- **Online Joint Control Interface**  
  Enables real-time commands of desired joint positions and velocities via the `/online_joint_control` topic, utilizing PID-based control.

- **Online TCP Control Interface**  
  Facilitates Cartesian pose control by processing target poses on the `/online_tcp_control` topic and computing appropriate joint velocities through the `JacobianController`.

- **Online Velocity Bypass Interface**  
  Accepts raw joint velocity commands on the `/online_velocity_bypass` topic and applies them directly to the robot, bypassing any internal control computations.


To ensure safe operation, all interfaces enforce mutual exclusivity, preventing command conflicts, and implement timeouts to automatically release control if command streams cease.


---

## Control Loop and Controller Management

The robot arm framework runs a dedicated real-time control loop in its own thread. This loop orchestrates sensing, control computation, and actuation, ensuring smooth and responsive arm motions. Each iteration of the loop performs the following steps:

1. **Check Active Interface Validity (Timeout Handling)**  
   For online control interfaces (joint control, TCP control, velocity bypass), the loop checks if commands have been received recently. If no new commands arrive within a preset timeout (e.g., 1 second), the system safely releases control by setting joint velocities to zero and marking the interface as inactive. This prevents runaway motions due to lost or stale commands.

2. **Update Controller Inputs**  
   Depending on the active controller type, the loop updates the controller with the latest state and command inputs:
   - For the PID controller, it computes and passes position and velocity errors, as well as desired joint velocities.
   - For the Jacobian controller, it provides the current joint positions and the desired TCP transformation.
   - For the velocity bypass controller, it forwards desired joint velocities directly.

3. **Invoke Controller Compute Step**  
   The loop calls the active controller's `control()` method, which returns the commanded joint velocities. This call is thread-safe and protected by mutex locks to avoid race conditions on shared data.

4. **Command Saturation and Publishing**  
   Before sending commands to the actuators, the loop verifies that requested velocities are within predefined maximum limits for each joint. If any velocity exceeds its limit, commands are either clamped or zeroed to maintain safety. The loop then publishes the verified velocity commands to the ROS 2 topic that interfaces with the robot's hardware controllers.

5. **Sleep to Maintain Control Frequency**  
   The control loop calculates elapsed time per cycle and sleeps just enough to maintain the desired control frequency (~125 Hz). If computation overruns occur, warnings are logged.

### Controller Switching

Controllers map directly to the different control interfaces (e.g., trajectory following, online joint or TCP control, velocity bypass). When a new interface requests control, the system:

- Checks if the interface is available (not occupied by another controller or currently active one).
- Stops the current control thread safely.
- Instantiates the requested controller from a factory map.
- Initializes the new controller (loading parameters if needed).
- Restarts the control loop with the newly selected controller.

This dynamic switching ensures seamless transitions between different modes of operation, preserving safety and system consistency.

---

## Usage

### Building and Running

- The project is built as a ROS 2 package and launched via the provided `launch.py` files.
- You must provide the robot's IP address and optionally tune controller parameters via YAML files (e.g., PID gains in `arm_right_ur5_pid_params.yaml`).
- The ROS 2 node accepts a configurable namespace prefix (e.g., `arm_right` or `arm_left`) to support multiple arms.

You can run all the necessary code using the following command:

```
ros2 launch robot_arm <robot_arm_name>.launch.py
```

Once running, you can send joint or pose paths (under your own risk) to move the robot using the client node:

```
ros2 run robot_arm robot_arm_client_node <robot_arm_name>
```

To add new paths, simply add a new CSV file in the appropriate folder. The system automatically detects new files. Alternatively, you can use the client code within your own applications for more customized control.



### Interacting With the Arm

- Publish joint trajectories to the `/prefix/move` action server.
- Publish desired joint states to the `/prefix/online_joint_control` topic.
- Publish desired TCP poses to the `/prefix/online_tcp_control` topic.
- Publish direct joint velocity commands to the `/prefix/online_velocity_bypass` topic.

### Extending Controllers

- Implement new controllers by inheriting from `RobotArmController` and registering them in the factory map.
- Ensure your controller provides initialization and control logic respecting kinematic order and thread safety.
- Add ROS 2 interface logic if new modes are required.


---

## Dependencies

This project depends on several external repositories developed at the IOC that provide essential low-level drivers and kinematics solvers specifically for Universal Robots (UR) manipulators. These must be cloned and built in your ROS 2 workspace before building this package:

```
git clone https://gitioc.upc.edu/robots/ioc_ur_cb2_driver.git
git clone https://gitioc.upc.edu/robots/ioc_ur_cb2_driver_ros.git -b ros2-jazzy
git clone https://gitioc.upc.edu/robots/kinenik.git -b refactoring
git clone https://gitioc.upc.edu/robots/robot_arm_interfaces.git
git clone https://gitioc.upc.edu/robots/ttg.git
```

- `ioc_ur_cb2_driver`: Provides low-level hardware interface drivers for UR robots.  
- `ioc_ur_cb2_driver_ros`: ROS 2 integration layer for the IOC UR driver, facilitating communication with ROS nodes and topics.  
- `kinenik`: Contains the kinematics solver (forward and inverse) used in this project. The `refactoring` branch includes latest improvements.
- `robot_arm_interfaces`: This package contains the custom interfaces that robot_arm uses.
- `ttg`: Time Trajectory Generator (TTG) is a ROS 2 node designed to generate time-parameterized joint trajectories for robotic manipulators. Used by the clients tests.

These dependencies together enable this robot arm framework to interface effectively with UR manipulators using ROS 2.

> **Note:** This framework is designed generically enough that it could also be adapted to work with the official Universal Robots ROS 2 driver or other ROS 2 control drivers (`ros2_control`) supporting different robot arms. Integrating alternative drivers would primarily involve adapting the hardware interface and kinematics solver layers.

---

## Current Limitations and Future Work

- Most components are designed to be **generic and extensible**, with clear abstraction layers.  
- Currently, some parts remain **hardcoded** to the Universal Robot UR5 model for convenience and compatibility. 
- Future work includes **removing hardcoded UR-specific elements** and enabling full support for other robot arms by adapting the kinematics and driver interface.



# 2. MADAR Synchronization and Coordination Framework

## Overview

A ROS 2 package designed to coordinate and synchronize the execution of trajectories across multiple robotic subsystems, primarily utilizing two instances of the robot_arm framework for controlling dual Universal Robots UR5 arms. The package manages joint trajectory execution for the coordinated movement of the robot's right and left manipulators, with provisional support for hands and a mobile platform.

It acts as a high-level manager node that encapsulates control, state, and communication for integrated robot components, offering action servers to manage complex motion goals seamlessly.


## Features

- Integration of multiple robotic subsystems under a single ROS 2 node.
- Support for coordinated trajectory execution for two UR5 arms.
- Modular initialization of subsystems based on configuration parameters.
- Action servers for combined control groups such as manipulator_right, dual_arm, and dual_manipulator.
- Thread-safe state management and feedback publication.
- Configurable via YAML parameters for flexibility in deployment.
- Uses the robust robot_arm framework for low-level arm control, including kinematics and PID controllers.


## Installation and Build

Create a ROS 2 workspace and clone the following dependencies along with `madar_robot`:

```
mkdir -p ws_madar/src
cd ws_madar/src
git clone https://gitioc.upc.edu/robots/ttg.git
git clone https://gitioc.upc.edu/robots/madar_robot.git -b ros2
git clone https://gitioc.upc.edu/robots/madar_description.git -b ros2
```
```
mkdir arms
cd arms
git clone https://gitioc.upc.edu/robots/ioc_ur_cb2_driver.git
git clone https://gitioc.upc.edu/robots/ioc_ur_cb2_driver_ros.git -b ros2-jazzy
git clone https://gitioc.upc.edu/robots/kinenik.git -b refactoring
git clone https://gitioc.upc.edu/robots/robot_arm.git -b ros2
git clone https://gitioc.upc.edu/robots/robot_arm_interfaces.git
```


Build the workspace:
```
colcon build --symlink-install
``` 

Source the setup scripts before running:
```
source install/setup.bash
``` 


## Usage

Launch the complete MADAR robot system with:

```
ros2 launch madar_robot madar_bring_up.launch.py
```

This launch file configures and starts the subcomponents based on the YAML configuration `madar_config.yaml`, initializing the arms, hands, and platform modules as enabled.

To control the robot, send joint trajectories or pose goals to the corresponding action servers, such as:

- `/dual_arm/move` for coordinated dual-arm trajectories
- `/manipulator_right/move` and `/manipulator_left/move` for individual arm-hand group

Example to run a client node to send trajectories:
```
ros2 run madar_robot madar_<action_type>_client_node
```


You can add new trajectory CSV files in the designated folders and the system will detect and use them automatically.

## Configuration

Edit `config/madar_config.yaml` to enable or disable subsystems and set IP addresses, communication ports, and namespace prefixes.


## Architecture

- MadarRobot node manages multiple RobotArm instances, initializing them with specific offset transforms for base and TCP frames.
- Action servers implement goals cancellation, acceptance, execution, and feedback according to ROS 2 standards.
- The "dual_arm" action server coordinates synchronized execution of trajectories for both right and left arms with PID control, ensuring safety and timing adherence.
- Threading and mutex locking guarantee consistent joint state updates and prevent race conditions.
- The current focus is on motion execution of joint trajectories; online TCP or velocity bypass modes are delegated to underlying robot_arm nodes.

## Future Work

- Integration of hands and mobile platform with full action support.
- Improvements in fault handling and extended feedback mechanisms.
- Generalization to support heterogeneous robot configurations beyond UR5.


# 3. OAK-D Head

A ROS2 project of a robot head, that incorporates the WidowX XM430 robot turret with an OAK-D camera, originally designed for MADAR robot.

It includes functionalities like:
- TF2 tracking
- Aruco tracking

## WidowX XM430 Robot Turret
The WidowX XM430 Robot Turret is a fully programmable pan and tilt system designed around the DYNAMIXEL servos from Robotis.The servos offer a high resolution of 4096 positions and user a definable PID parameters. Temperature monitoring, positionnal feedback as well as voltage levels, load and compliance settings are all user accessible as well.

## OAK-D camera
The OAK-D camera, which stands for OpenCV AI Kit with Depth, is a potent camera made for use in applications involving computer vision and artificial intelligence. It is based on Intel's Myriad X processor, a highly effective system-on-chip developed for computer vision and artificial intelligence applications. The stereo vision in the OAK-D camera enables it to record 3D depth information.

## Install and build the package
First create a workspace:
```
mkdir -p ws_oakd_head/src
cd ws_oakd_head/src
``` 
In addition to the current repository, the following repositories must be cloned:
```
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b ros2
git clone https://github.com/Interbotix/interbotix_xs_driver.git -b ros2
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git -b ros2
git clone https://github.com/Interbotix/interbotix_ros_core.git -b humble
git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git -b humble
git clone https://github.com/Interbotix/interbotix_ros_turrets.git -b humble
git clone https://gitioc.upc.edu/labs/aruco_broadcaster.git -b ros2
git clone https://gitioc.upc.edu/labs/tablesens.git -b ros2
```

To be able to detect arucos with the OAK-D camera, install the requeried dependency:
```
sudo apt install ros-humble-depthai-ros
sudo apt-get install ros-humble-aruco-ros
sudo apt-get install ros-humble-aruco-msgs
sudo apt-get install ros-humble-image-proc
```

Then, from ws_oakd_head/src:
```
rm ./interbotix_ros_core/interbotix_ros_xseries/COLCON_IGNORE
```

Finally, at the root of your ROS 2 workspace, build using:
```
colcon build --symlink-install
``` 

## Configuration instructions
Before running the programs, check that you have the ttyDXL port which is the one requested to be able to run the simulation programs, if you do not have this port, configure it.

First list the ports with the following command:
```
ls -l /dev/tty*
``` 
Then, if the USB is connected, you will be able to see a new port (that didn't appear before), this could be, for example, /dev/ttyACM0. Finally run the following command to create the link:
```
sudo ln -s /dev/ttyACM0 /dev/ttyDXL
``` 

## Calibration instructions
To locate automatically the pose of the camera base when is used as the head of the MADAR robot, the next commands must be used:
```
ros2 launch oakd_head locusifid_madar_oakd_head.launch.py
```
Then, write the output of the previous code (printed in the terminal), in the launch file madar_oakd_head.launch.py.

## Usage instructions
Once the code is compiled, and the workspace is set using:
```
source install/setup.bash
```
There are two available launch files:
- Basic OAK-D head control:
    ```
    ros2 launch oakd_head oakd_head.launch.py
    ```

- Basic OAK-D head control as a MADAR head (with aruco detection):
    ```
    ros2 launch oakd_head madar_oakd_head.launch.py
    ```
    - Then, set the aruco that you want to track using the following command from another terminal or create a client insite your top level code:
        ```
        ros2 service call /oakd_head/set_frame_tracker oakd_head/srv/SetFrameTracker "frame_name: 'aruco_301'"
        ```
    - Finally, to stop the tracking:
        ```
        ros2 service call /oakd_head/set_frame_tracker oakd_head/srv/SetFrameTracker "frame_name: ''"
        ```

## Test instructions
To simulate transformations you can just run the following node:
```
ros2 run oakd_head random_tf_broadcaster
```
Or run the follwing commands:
- Fisrt, create the transformation:
    ```
    ros2 run tf2_ros static_transform_publisher --x 1 --y 0 --z 0.2 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id wxxms/base_link --child-frame-id this_frame
    ```
- Then, set the frame name to track:
    ```
    ros2 service call /oakd_head/set_frame_tracker oakd_head/srv/SetFrameTracker "frame_name: 'this_frame'"
    ```
- Finally, to stop the tracking:
    ```
    ros2 service call /oakd_head/set_frame_tracker oakd_head/srv/SetFrameTracker "frame_name: ''"
    ```
