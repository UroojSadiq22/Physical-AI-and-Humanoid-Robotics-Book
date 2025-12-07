---
title: Building a Robot with URDF
sidebar_position: 2
---

## Chapter 6: Building a Robot with URDF

### Learning Objectives

-   Understand the purpose and structure of URDF (Unified Robot Description Format).
-   Learn how to define robot links and joints using URDF.
-   Create a simple URDF model for a robotic arm or wheeled robot.
-   Visualize URDF models using `rviz2`.

### Scope

This chapter covers the fundamentals of URDF, a standard XML format for describing robots in ROS. We will learn to define the robot's physical structure, kinematic and dynamic properties, and visual appearance.

### Key Concepts

-   URDF (Unified Robot Description Format)
-   Link (rigid body)
-   Joint (connection between links)
-   `robot` tag
-   `link` tag (visual, collision, inertial)
-   `joint` tag (type, axis, origin, parent, child)
-   `rviz2` (ROS Visualization)

### Practical Components

-   Designing a basic robotic arm or mobile robot structure.
-   Writing a URDF file from scratch.
-   Adding visual, collision, and inertial properties to links.
-   Defining different types of joints (fixed, revolute, continuous).
-   Loading and visualizing the URDF model in `rviz2`.

### Expected Outputs

-   A well-formed URDF file describing a simple robot.
-   The ability to visualize the robot's structure and joint hierarchy in `rviz2`.

## Introduction to URDF

When working with robots in simulation or even with physical hardware, it's essential to have a consistent and comprehensive way to describe the robot's physical and kinematic properties. This is where **URDF (Unified Robot Description Format)** comes into play.

URDF is an XML format used in ROS to describe all aspects of a robot model. It allows you to specify:
-   **Links**: The rigid bodies of the robot (e.g., base, arm segments, wheels).
-   **Joints**: The connections between these links, defining their type of motion (e.g., revolute, prismatic, fixed).
-   **Kinematic and Dynamic Properties**: How the robot moves and reacts to forces.
-   **Visual and Collision Properties**: How the robot looks and how it interacts with its environment.

### Basic URDF Structure

A URDF file always starts with a `<robot>` tag, which contains one or more `<link>` and `<joint>` tags.

```xml
<?xml version="1.0"?>
<robot name="my_simple_robot">

  <!-- Define Links Here -->
  <link name="base_link">
    <!-- Visual, Collision, Inertial properties -->
  </link>

  <link name="link1">
    <!-- Visual, Collision, Inertial properties -->
  </link>

  <!-- Define Joints Here -->
  <joint name="base_to_link1_joint" type="revolute">
    <!-- Origin, Axis, Parent, Child -->
  </joint>

</robot>
```

### Links: The Robot's Body Parts

A `<link>` tag describes a rigid body of the robot. Each link has a name and can have three main properties:

1.  **`<visual>`**: Defines how the link looks. This typically includes a geometry (e.g., box, cylinder, mesh) and material properties (color).
    ```xml
    <link name="base_link">
      <visual>
        <geometry>
          <box size="0.2 0.2 0.1"/>
        </geometry>
        <material name="blue">
          <color rgba="0 0 1 1"/>
        </material>
      </visual>
    </link>
    ```

2.  **`<collision>`**: Defines the link's geometry for collision detection. This is often the same as the visual geometry but can be simplified for computational efficiency.
    ```xml
    <link name="base_link">
      <collision>
        <geometry>
          <box size="0.2 0.2 0.1"/>
        </geometry>
      </collision>
    </link>
    ```

3.  **`<inertial>`**: Defines the link's mass properties (mass, center of mass, inertia tensor). These are crucial for accurate physics simulation.
    ```xml
    <link name="base_link">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 0"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>
    ```

### Joints: Connecting the Links

A `<joint>` tag describes how two links are connected. Key attributes include:

-   **`name`**: Unique identifier for the joint.
-   **`type`**: Specifies the joint's degrees of freedom. Common types include:
    -   `revolute`: Rotates around a single axis, with upper and lower limits.
    -   `continuous`: Rotates around a single axis continuously (no limits).
    -   `prismatic`: Translates along a single axis, with upper and lower limits.
    -   `fixed`: No movement; rigidly connects two links.
-   **`<origin>`**: Defines the transform from the parent link's origin to the joint's origin.
-   **`<parent link="..." />`**: The name of the parent link.
-   **`<child link="..." />`**: The name of the child link.
-   **`<axis xyz="X Y Z" />`**: The axis of rotation or translation for revolute, continuous, and prismatic joints.
-   **`<limit lower="..." upper="..." effort="..." velocity="..." />`**: Defines the joint limits for revolute and prismatic joints.

#### Example: A Simple 2-Link Arm

Let's create a URDF for a simple 2-link robotic arm.

**1. Create a package for your robot description:**

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_description
```

**2. Create the URDF file:**

Create `~/ros2_ws/src/my_robot_description/urdf/simple_arm.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
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
    <origin xyz="0 0 0.025" rpy="0 0 0"/> <!-- Offset to top of base_link -->
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/> <!-- Offset visual to center of link -->
      <geometry>
        <cylinder radius="0.02" length="0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
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
    <origin xyz="0 0 0.3" rpy="0 0 0"/> <!-- Offset to top of link1 -->
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Link 2 (End-effector) -->
  <link name="link2">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/> <!-- Offset visual to center of link -->
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

**3. Update `package.xml`**

Add `xacro` and `joint_state_publisher_gui` dependencies for visualization later:

```xml
  <depend>xacro</depend>
  <depend>joint_state_publisher_gui</depend>
```

**4. Update `CMakeLists.txt`**

To install the URDF file:

```cmake
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)
```

**5. Build the package:**

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
```

### Visualizing URDF with `rviz2`

`rviz2` is the primary 3D visualization tool for ROS 2. We can use it to visualize our URDF model.

**1. Launch `rviz2`:**

```bash
rviz2
```

**2. Add the `RobotStatePublisher` and `JointStatePublisher`:**

To properly visualize the robot and control its joints, you need to run `robot_state_publisher` and `joint_state_publisher_gui`.

Create a launch file `~/ros2_ws/src/my_robot_description/launch/display.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
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

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # Rviz2
    rviz_config_file = os.path.join(get_package_share_directory('my_robot_description'), 'rviz', 'urdf_config.rviz')
    # Create a dummy rviz config file if it doesn't exist
    if not os.path.exists(rviz_config_file):
        os.makedirs(os.path.dirname(rviz_config_file), exist_ok=True)
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
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
```

**3. Launch the display nodes:**

```bash
ros2 launch my_robot_description display.launch.py
```

In `rviz2`, add a "RobotModel" display and set its "Description Source" to "Topic" and "Update Topic" to `/robot_description`. You should see your arm model. The `joint_state_publisher_gui` will provide sliders to move the joints.

In this chapter, we've laid the groundwork for defining our robot's physical structure using URDF, a crucial step for both simulation and real-world robot operation.
