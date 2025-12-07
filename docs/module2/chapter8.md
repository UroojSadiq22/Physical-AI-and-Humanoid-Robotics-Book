---
title: Simulating a Robot with Unity
sidebar_position: 4
---

## Chapter 8: Simulating a Robot with Unity

### Learning Objectives

-   Understand the advantages of using Unity for robotic simulation, especially for visual fidelity and AI training.
-   Learn how to set up a Unity project for robotics simulation.
-   Integrate URDF models into Unity using the Unity Robotics Hub packages.
-   Establish communication between Unity and ROS 2.

### Scope

This chapter explores Unity as a powerful, visually rich platform for robotic simulation. We will cover the installation of necessary Unity packages, importing URDF models, and setting up bi-directional communication with ROS 2.

### Key Concepts

-   Unity 3D Engine
-   Unity Robotics Hub
-   ROS-TCP-Connector
-   ROS-Unity-Integration
-   Visual Fidelity Simulation
-   AI Training in Simulation

### Practical Components

-   Setting up a new Unity project for robotics.
-   Importing and configuring the Unity Robotics Hub packages.
-   Importing a URDF model into Unity.
-   Setting up a ROS-TCP-Connector to enable communication with ROS 2.
-   Sending sensor data from Unity to ROS 2.
-   Receiving control commands from ROS 2 to actuate a simulated robot in Unity.

### Expected Outputs

-   A Unity scene with a simulated robot loaded from a URDF.
-   Real-time communication between the Unity simulation and ROS 2 nodes.
-   Ability to control the Unity robot from ROS 2 and visualize sensor data in ROS 2 tools.

## Introduction to Unity for Robotics Simulation

Unity, primarily known as a game development engine, has emerged as a powerful platform for robotic simulation. Its key strengths lie in:

-   **High Visual Fidelity**: Create photorealistic environments for advanced sensor simulation (e.g., cameras, LiDAR) and human-robot interaction studies.
-   **Extensive Asset Store**: Access a vast library of 3D models, environments, and tools.
-   **Physics Engine (NVIDIA PhysX)**: Provides robust and accurate physics simulation.
-   **AI Training Capabilities**: Ideal for reinforcement learning and generating synthetic data for computer vision models, especially when integrated with tools like NVIDIA Isaac Sim (built on Unity).

The **Unity Robotics Hub** provides a collection of packages and tools to streamline the integration of Unity with robotics frameworks like ROS.

### Setting Up Your Unity Project

#### 1. Install Unity Hub and Unity Editor

Download and install Unity Hub, then install a compatible Unity Editor version (e.g., Unity 2021.3 LTS or newer).

#### 2. Create a New 3D Project

Launch Unity Hub, create a new 3D project, and give it a suitable name (e.g., `UnityRobotSim`).

#### 3. Import Unity Robotics Packages

Open your new Unity project. Navigate to `Window > Package Manager`.
-   Click the `+` icon, then `Add package from git URL...`
-   Add the following packages one by one (check Unity Robotics Hub documentation for latest versions/URLs):
    -   `https://github.com/Unity-Technologies/Robotics-Utilities.git`
    -   `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`
    -   `https://github.com/Unity-Technologies/ROS-Unity-Integration.git`
    -   `https://github.com/Unity-Technologies/Unity-Robotics-URDF-Importer.git`

After importing, you might need to restart Unity for some changes to take effect.

### Importing a URDF Model into Unity

The `Unity-Robotics-URDF-Importer` package allows you to easily bring your robot descriptions (URDF files) into Unity.

#### 1. Prepare Your URDF (Optional: Xacro)

If your URDF is an Xacro file, convert it to a standard URDF first:

```bash
ros2 run xacro xacro --inorder -o simple_arm.urdf simple_arm.urdf.xacro # Replace with your xacro file
```

#### 2. Import into Unity

-   In Unity, go to `Robotics > URDF Importer > Import URDF`.
-   Browse to your URDF file (e.g., `~/ros2_ws/src/my_robot_description/urdf/simple_arm.urdf`).
-   Configure import settings (e.g., import visuals, collisions, physics).
-   Click `Import`.

Your robot model should now appear in your Unity project's `Assets` folder and can be dragged into your scene.

### Establishing ROS 2 Communication (ROS-TCP-Connector)

The `ROS-TCP-Connector` enables communication between your Unity simulation and ROS 2 nodes running on your system (or a remote system).

#### 1. Configure ROS-TCP-Connector in Unity

-   In your Unity scene, create an empty GameObject and name it `ROSConnection`.
-   Add the `ROSConnection` script component to this GameObject.
-   Configure the `ROS IP Address` and `ROS Port` in the inspector (default is usually `127.0.0.1` and `10000`).

#### 2. Run ROS-TCP-Endpoint

On your ROS 2 system, run the `ros_tcp_endpoint` node:

```bash
ros2 run ros_tcp_endpoint default_rviz.launch.py
```

This node creates the TCP server that Unity connects to.

### Sending Sensor Data from Unity to ROS 2

Let's simulate a simple IMU sensor in Unity and publish its data to ROS 2.

#### 1. Create a ROS 2 Message Publisher in Unity

-   Attach a script (e.g., `ImuPublisher.cs`) to your robot's IMU link (or a dedicated sensor GameObject).
-   In the script, import `RosMessageTypes.Sensor` and `Unity.Robotics.ROSTCPConnector`.
-   Use `rosConnection.RegisterPublisher<ImuMsg>("imu_topic");` to register a publisher.
-   In `Update()`, populate an `ImuMsg` with simulated data (e.g., random values for acceleration/angular velocity) and publish it using `rosConnection.Publish("imu_topic", imuMsg);`.

Example (simplified `ImuPublisher.cs`):

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // Assuming you have sensor_msgs imported

public class ImuPublisher : MonoBehaviour
{
    ROSConnection rosConnection;
    public string topicName = "imu_data";
    public float publishMessageFrequency = 0.5f;

    private float timeElapsed;

    void Start()
    {
        rosConnection = ROSConnection.Get  Instance();
        rosConnection.RegisterPublisher<ImuMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)
        {
            ImuMsg imu = new ImuMsg();
            // Populate IMU data (e.g., from Unity's Input.gyro or simulated values)
            imu.header.frame_id = "imu_link";
            imu.linear_acceleration.x = Random.Range(-1.0f, 1.0f);
            imu.angular_velocity.z = Random.Range(-0.5f, 0.5f);
            // ... fill other fields ...

            rosConnection.Publish(topicName, imu);
            timeElapsed = 0;
        }
    }
}
```

#### 2. Create a ROS 2 Subscriber Node (Python)

On your ROS 2 system, create a Python node to subscribe to the IMU data (similar to Chapter 3's subscriber).

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu # Make sure sensor_msgs is available

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Imu):
        self.get_logger().info(
            f'Received IMU data - Accel: ({msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}) '
            f'Angular Vel: ({msg.angular_velocity.x:.2f}, {msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Receiving Control Commands from ROS 2 to Unity

Now, let's control a simulated joint in Unity from ROS 2.

#### 1. Create a ROS 2 Command Subscriber in Unity

-   Attach a script (e.g., `JointController.cs`) to your robot's root GameObject.
-   Register a subscriber: `rosConnection.RegisterSubscriber<JointStateMsg>("joint_commands", OnJointCommandsReceived);`
-   In `OnJointCommandsReceived`, parse the message and apply the commands to the corresponding joints in your Unity robot model (e.g., by setting `articulationBody.SetDriveTarget()`).

Example (simplified `JointController.cs`):

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // For JointStateMsg
using System.Collections.Generic;

public class JointController : MonoBehaviour
{
    ROSConnection rosConnection;
    public string topicName = "joint_commands";
    public List<ArticulationBody> joints; // Assign your robot's joints here in inspector

    void Start()
    {
        rosConnection = ROSConnection.Get  Instance();
        rosConnection.RegisterSubscriber<JointStateMsg>(topicName, OnJointCommandsReceived);
    }

    void OnJointCommandsReceived(JointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Length; i++)
        {
            string jointName = jointState.name[i];
            float targetPosition = (float)jointState.position[i];

            // Find the corresponding joint in your Unity robot
            ArticulationBody targetJoint = joints.Find(j => j.name == jointName);
            if (targetJoint != null)
            {
                ArticulationDrive drive = targetJoint.xDrive;
                drive.target = targetPosition * Mathf.Rad2Deg; // Convert radians to degrees if needed
                targetJoint.xDrive = drive;
                Debug.Log($"Setting joint {jointName} to {targetPosition}");
            }
        }
    }
}
```

#### 2. Create a ROS 2 Publisher Node (Python)

On your ROS 2 system, create a Python node to publish joint commands.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # To send joint commands

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_commands', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2'] # Names must match your Unity joint names
        self.angle += 0.1
        if self.angle > 1.0:
            self.angle = -1.0
        msg.position = [self.angle, -self.angle]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing JointState: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    joint_command_publisher = JointCommandPublisher()
    rclpy.spin(joint_command_publisher)
    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Conclusion

Unity provides a powerful and flexible environment for robotic simulation, especially when visual realism and AI training are paramount. By leveraging the Unity Robotics Hub and the `ROS-TCP-Connector`, you can seamlessly integrate your Unity simulations with the ROS 2 ecosystem, enabling sophisticated control and data exchange between your virtual robot and ROS 2 applications.
