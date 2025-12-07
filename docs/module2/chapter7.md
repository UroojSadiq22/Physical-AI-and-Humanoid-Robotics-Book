---
title: Simulating a Robot with Gazebo
sidebar_position: 3
---

## Chapter 7: Simulating a Robot with Gazebo

### Learning Objectives

-   Understand the architecture and key components of Gazebo.
-   Learn how to launch Gazebo and spawn URDF models.
-   Interact with the simulated robot and environment.
-   Integrate Gazebo with ROS 2 for sensor data and motor control.

### Scope

This chapter provides a hands-on guide to using Gazebo for robotic simulation. We will cover environment setup, loading URDF models, basic interaction, and the crucial integration with ROS 2.

### Key Concepts

-   Gazebo Simulator
-   World File (`.world`)
-   Model File (`.sdf`, `.urdf`)
-   Gazebo ROS 2 Packages (`gazebo_ros_pkgs`)
-   Gazebo Plugins (sensor, joint state, differential drive)
-   `ros2_control`

### Practical Components

-   Launching an empty Gazebo world.
-   Spawning a simple URDF robot model (e.g., from Chapter 6).
-   Adding basic sensors (e.g., LiDAR, camera) to the URDF and simulating them in Gazebo.
-   Implementing a simple ROS 2 node to control a simulated robot's joints or wheels.
-   Visualizing simulated sensor data in `rviz2`.

### Expected Outputs

-   A running Gazebo simulation with a spawned URDF robot.
-   A ROS 2 node that can send commands to and receive data from the simulated robot.
-   Real-time visualization of simulated sensor data.

## Introduction to Gazebo

**Gazebo** is a powerful 3D robotics simulator widely used in the ROS community. It can accurately simulate robots, sensors, and environments, providing a realistic testbed for developing and debugging robotic applications. Gazebo uses a robust physics engine (historically ODE, but also supports Bullet, DART, Simbody) to provide accurate dynamics.

### Gazebo Architecture Overview

Gazebo consists of several key components:

-   **Gazebo Server (`gzserver`)**: The core physics engine and simulation logic, running headless (without a GUI).
-   **Gazebo Client (`gzclient`)**: The graphical user interface (GUI) that allows you to visualize the simulation, interact with models, and debug.
-   **World Files (`.world`)**: XML files that define the static environment (e.g., ground plane, walls, furniture) and initial placement of robot models and sensors.
-   **Model Files (`.sdf`, `.urdf`)**: Define individual robot models or objects. While Gazebo natively uses SDF (Simulation Description Format), it can import and use URDF files, converting them to SDF internally.
-   **Plugins**: Extend Gazebo's functionality, allowing for custom sensor models, motor controllers, or interaction logic. Gazebo ROS 2 packages provide a bridge between Gazebo and ROS 2 using plugins.

### Launching Gazebo and Spawning a URDF Model

Let's launch Gazebo and load the simple arm URDF model we created in Chapter 6.

#### 1. Ensure ROS 2 and Gazebo are Installed

Follow the official ROS 2 and Gazebo installation guides for Ubuntu 22.04 if you haven't already.

#### 2. Create a Gazebo Launch File

Modify `~/ros2_ws/src/my_robot_description/launch/display.launch.py` to also launch Gazebo.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true', # Set to true for simulation
            description='Use simulation (Gazebo) clock if true',
        )
    )

    # Get URDF file
    urdf_file_name = 'simple_arm.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        urdf_file_name
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_path, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Joint State Publisher GUI (optional, useful for manual control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=LaunchConfiguration('use_sim_time'), # Only run if use_sim_time is true
    )

    # Rviz2 (optional, useful for visualization)
    rviz_config_dir = os.path.join(get_package_share_directory('my_robot_description'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'urdf_config.rviz')
    # Ensure the rviz config directory exists
    os.makedirs(rviz_config_dir, exist_ok=True)
    # Create a dummy rviz config file if it doesn't exist (or overwrite)
    with open(rviz_config_file, 'w') as f:
        f.write("""
Global Options:
  Fixed Frame: base_link
Displays:
  - Class: rviz_default_plugins/RobotModel
    Name: RobotModel
    Enabled: true
    Visual Enabled: true
    Collision Enabled: false
    Inertia Enabled: false
    Description Source: Topic
    Update Topic: /robot_description
  - Class: rviz_default_plugins/JointState
    Name: JointState
    Enabled: true
    Show Names: true
    Show Axes: true
    Show Arrows: true
    Show Handles: true
    Topic: /joint_states
""")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=LaunchConfiguration('use_sim_time'), # Only run if use_sim_time is true
    )

    # Gazebo launch
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    
    # Spawn the URDF model in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-file', urdf_path,
                                   '-entity', 'simple_arm',
                                   '-x', '0', '-y', '0', '-z', '0.1'], # Spawn slightly above ground
                        output='screen')

    return LaunchDescription(declared_arguments + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': 'empty.world'}.items(), # Use an empty world
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        spawn_entity,
    ])
```

#### 3. Build and Launch

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
ros2 launch my_robot_description display.launch.py
```

You should see Gazebo launching with an empty world and your simple arm robot spawned in it. In `rviz2`, you can still visualize the robot and control its joints with the GUI.

### Integrating ROS 2 with Gazebo using Plugins

To make the simulated robot interact with ROS 2, we need to add Gazebo plugins to our URDF. These plugins publish sensor data to ROS 2 topics and subscribe to ROS 2 topics for motor commands.

#### 1. Add Gazebo Plugins to URDF

Let's modify `~/ros2_ws/src/my_robot_description/urdf/simple_arm.urdf` to include a simple joint state publisher plugin and a differential drive plugin (if it were a mobile robot, for arm control we might use `ros2_control` plugins which are more complex). For our arm, a joint state publisher is sufficient to get joint positions.

First, add the `gazebo` tag to the `robot` tag, and include `gazebo_ros` package in `package.xml`:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>

  <!-- Include the common Gazebo ROS 2 plugins for a simple arm -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find my_robot_description)/config/arm_controller.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint 1: Base to Link 1 -->
  <joint name="joint1" type="revolute">
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.3"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.00005" ixy="0" ixz="0" iyy="0.00005" iyz="0" izz="0.000001"/>
    </inertial>
  </link>

  <!-- Joint 2: Link 1 to Link 2 -->
  <joint name="joint2" type="revolute">
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Link 2 (End-effector) -->
  <link name="link2">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <origin xyz="0 0 0.05"/>
      <inertia ixx="0.000005" ixy="0" ixz="0" iyy="0.000005" iyz="0" izz="0.000005"/>
    </inertial>
  </link>

</robot>
```

Add to `~/ros2_ws/src/my_robot_description/package.xml`:

```xml
  <depend>gazebo_ros</depend>
  <depend>ros2_control</depend>
  <depend>controller_manager</depend>
```

Create `~/ros2_ws/src/my_robot_description/config/arm_controller.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

      joints:
        - joint1
        - joint2
      
      command_interfaces:
        - position

      state_interfaces:
        - position
        - velocity
```

#### 2. Update the Launch File for `ros2_control`

Modify `~/ros2_ws/src/my_robot_description/launch/display.launch.py` to launch the `ros2_control` nodes:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
import xacro # To parse xacro if needed, though for URDF it's direct

def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true', # Set to true for simulation
            description='Use simulation (Gazebo) clock if true',
        )
    )

    # Get URDF file and parse it
    urdf_file_name = 'simple_arm.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        urdf_file_name
    )

    # If your URDF is a xacro file, you would parse it here:
    # doc = xacro.parse(open(urdf_path))
    # xacro.process_doc(doc)
    # robot_desc = doc.toprettyxml(indent='  ')
    # For plain URDF, read directly:
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Joint State Publisher GUI (optional, useful for manual control if not using ros2_control)
    # This might conflict with ros2_control if both try to publish joint states
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen',
    #     condition=LaunchConfiguration('use_sim_time'),
    # )

    # Rviz2 (optional, useful for visualization)
    rviz_config_dir = os.path.join(get_package_share_directory('my_robot_description'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'urdf_config.rviz')
    os.makedirs(rviz_config_dir, exist_ok=True)
    with open(rviz_config_file, 'w') as f: # Overwrite to ensure it's up to date
        f.write("""
Global Options:
  Fixed Frame: base_link
Displays:
  - Class: rviz_default_plugins/RobotModel
    Name: RobotModel
    Enabled: true
    Visual Enabled: true
    Collision Enabled: false
    Inertia Enabled: false
    Description Source: Topic
    Update Topic: /robot_description
  - Class: rviz_default_plugins/JointState
    Name: JointState
    Enabled: true
    Show Names: true
    Show Axes: true
    Show Arrows: true
    Show Handles: true
    Topic: /joint_states
""")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=LaunchConfiguration('use_sim_time'),
    )

    # Gazebo launch
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    
    # Spawn the URDF model in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-file', urdf_path,
                                   '-entity', 'simple_arm',
                                   '-x', '0', '-y', '0', '-z', '0.1'],
                        output='screen')

    # Load and Start ros2_control controllers
    robot_controller_config = os.path.join(
        get_package_share_directory('my_robot_description'),
        'config',
        'arm_controller.yaml'
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': LaunchConfiguration('use_sim_time')}, robot_controller_config],
        output='screen',
    )

    # Automatically start the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Automatically start the joint trajectory controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )


    return LaunchDescription(declared_arguments + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': 'empty.world'}.items(),
        ),
        robot_state_publisher_node,
        rviz_node,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        control_node,
    ])
```

#### 3. Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
ros2 launch my_robot_description display.launch.py
```

Now, in Gazebo, you should see your robot. You can then use ROS 2 commands to interact with its controllers. For instance, to send a joint trajectory command:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
joint_names:
  - joint1
  - joint2
points:
  - positions: [1.0, 0.5]
    velocities: [0.0, 0.0]
    accelerations: [0.0, 0.0]
    time_from_start: {sec: 2, nanosec: 0}"
```

This command would attempt to move `joint1` to 1.0 radian and `joint2` to 0.5 radian over 2 seconds.

### Conclusion

Gazebo provides a robust environment for simulating complex robotic systems. By integrating it with ROS 2 through plugins, we can develop and test our robot control software in a safe and reproducible virtual world before deploying it to physical hardware.
