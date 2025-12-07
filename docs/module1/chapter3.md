---
title: ROS 2 Topics
sidebar_position: 3
---

## Chapter 3: ROS 2 Topics

### Learning Objectives

-   Understand the concept of ROS 2 topics for inter-node communication.
-   Learn how to publish data to a topic.
-   Learn how to subscribe to data from a topic.
-   Explore common ROS 2 message types.

### Scope

This chapter delves into ROS 2 topics, the primary mechanism for asynchronous, many-to-many data streaming between nodes. We will cover how to define, publish, and subscribe to topics using practical Python examples.

### Key Concepts

-   ROS 2 Topic
-   Publisher
-   Subscriber
-   Message types (`std_msgs`, custom messages)
-   Quality of Service (QoS) settings

### Practical Components

-   Creating a custom ROS 2 message.
-   Implementing a publisher node that sends custom messages over a topic.
-   Implementing a subscriber node that receives and processes custom messages.
-   Using `ros2 topic` command-line tools to inspect topic activity.

### Expected Outputs

-   A working custom ROS 2 message definition.
-   Python nodes that successfully communicate using custom messages over a topic.
-   Ability to inspect topic data using `ros2 topic echo`.

## Introduction to ROS 2 Topics

ROS 2 topics are a fundamental concept for building distributed robotic systems. They provide a mechanism for different nodes to exchange data asynchronously. A node can "publish" data to a named topic, and any other node can "subscribe" to that topic to receive the data. This publish/subscribe model allows for flexible and decoupled communication between various components of a robot.

### Custom Message Definition

While ROS 2 provides standard message types (like `std_msgs/String`, `geometry_msgs/Twist`), you often need to define custom messages to suit your application's specific data requirements.

#### 1. Create a Custom Message Package

In your workspace, create a new package for custom messages.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake custom_interfaces
```

#### 2. Define Your Message

Create a new file `~/ros2_ws/src/custom_interfaces/msg/RobotStatus.msg` with the following content:

```
# RobotStatus.msg
# Represents the current status of a robot

builtin_interfaces/msg/Time timestamp
string robot_name
float32 battery_percentage
bool is_charging
uint8 status_code # 0: IDLE, 1: MOVING, 2: ERROR
```

#### 3. Update `package.xml`

Add these lines to `custom_interfaces/package.xml`:

```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

#### 4. Update `CMakeLists.txt`

Modify `~/ros2_ws/src/custom_interfaces/CMakeLists.txt` to enable message generation:

```cmake
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

#### 5. Build the Custom Message

Build your workspace to generate the necessary code for your custom message:

```bash
cd ~/ros2_ws
colcon build --packages-select custom_interfaces
source install/setup.bash
```

### Implementing Publisher and Subscriber for Custom Messages

Now, let's update our `my_robot_controller` package to use the `RobotStatus` message.

#### 1. Update `package.xml` for `my_robot_controller`

Add a dependency on `custom_interfaces` in `~/ros2_ws/src/my_robot_controller/package.xml`:

```xml
  <depend>custom_interfaces</depend>
```

#### 2. Custom Publisher Node

Create `~/ros2_ws/src/my_robot_controller/my_robot_controller/status_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import RobotStatus
from builtin_interfaces.msg import Time

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.battery = 100.0
        self.is_charging = False
        self.status_code = 0 # IDLE

    def timer_callback(self):
        msg = RobotStatus()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.robot_name = "RoboChimp-001"
        msg.battery_percentage = self.battery
        msg.is_charging = self.is_charging
        msg.status_code = self.status_code

        self.battery -= 0.5
        if self.battery < 20.0:
            self.is_charging = True
        if self.battery > 95.0:
            self.is_charging = False
            self.battery = 95.0 # Prevent overcharge

        if self.is_charging:
            self.battery += 1.0
            self.status_code = 0 # IDLE while charging
        elif self.battery < 10.0:
            self.status_code = 2 # ERROR
        else:
            self.status_code = (self.status_code + 1) % 2 # Toggle between IDLE and MOVING

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing RobotStatus: {msg.robot_name}, Battery: {msg.battery_percentage:.1f}%, Charging: {msg.is_charging}, Status: {msg.status_code}')

def main(args=None):
    rclpy.init(args=args)
    status_publisher = StatusPublisher()
    rclpy.spin(status_publisher)
    status_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 3. Custom Subscriber Node

Create `~/ros2_ws/src/my_robot_controller/my_robot_controller/status_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import RobotStatus

class StatusSubscriber(Node):
    def __init__(self):
        super().__init__('status_subscriber')
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: RobotStatus):
        status_map = {0: "IDLE", 1: "MOVING", 2: "ERROR"}
        self.get_logger().info(
            f'Received RobotStatus from {msg.robot_name}: '
            f'Battery: {msg.battery_percentage:.1f}%, '
            f'Charging: {msg.is_charging}, '
            f'Status: {status_map.get(msg.status_code, "UNKNOWN")}'
        )

def main(args=None):
    rclpy.init(args=args)
    status_subscriber = StatusSubscriber()
    rclpy.spin(status_subscriber)
    status_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 4. Update `setup.py`

Add the new executables to `~/ros2_ws/src/my_robot_controller/setup.py`:

```python
        'console_scripts': [
            'simple_publisher = my_robot_controller.simple_publisher:main',
            'simple_subscriber = my_robot_controller.simple_subscriber:main',
            'status_publisher = my_robot_controller.status_publisher:main',
            'status_subscriber = my_robot_controller.status_subscriber:main',
        ],
```

#### 5. Build and Run

Build your workspace again:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_controller
source install/setup.bash
```

Now run the nodes:

```bash
# In one terminal:
ros2 run my_robot_controller status_publisher

# In another terminal:
ros2 run my_robot_controller status_subscriber
```

You should see the subscriber receiving the custom `RobotStatus` messages.

### Using `ros2 topic` Command-Line Tools

ROS 2 provides powerful command-line tools to interact with topics.

-   **List active topics**:
    ```bash
    ros2 topic list
    ```
    You should see `/robot_status` and `/topic`.

-   **Echo topic messages**:
    ```bash
    ros2 topic echo /robot_status
    ```
    This will display the messages being published on the `/robot_status` topic in real-time.

-   **Get topic information**:
    ```bash
    ros2 topic info /robot_status
    ```
    This shows the message type and the number of publishers and subscribers for the topic.
