---
title: ROS 2 Nodes
sidebar_position: 2
---

## Chapter 2: ROS 2 Nodes

### Learning Objectives

- Understand the concept of a ROS 2 node.
- Learn how to create a simple ROS 2 node in Python.
- Understand how nodes communicate within a ROS 2 system.

### Scope

This chapter covers the fundamental concept of ROS 2 nodes, their role in a robotic system, and basic implementation in Python.

### Key Concepts

-   ROS 2 Node
-   rclpy library
-   Node initialization and shutdown
-   Node lifecycle

### Practical Components

-   Creating a "Hello World" ROS 2 publisher node.
-   Creating a "Hello World" ROS 2 subscriber node.
-   Running multiple nodes simultaneously.

### Expected Outputs

-   A functional ROS 2 publisher node that prints messages.
-   A functional ROS 2 subscriber node that receives and prints messages.

## Introduction to ROS 2 Nodes

In ROS 2, a node is an executable process that performs computations. Nodes are designed to be modular, meaning each node is responsible for a single, well-defined task (e.g., controlling a motor, reading sensor data, performing navigation). This modularity allows for the creation of complex robotic systems by combining many simple nodes.

### Creating Your First ROS 2 Node (Python)

To create a ROS 2 node in Python, we use the `rclpy` client library.

#### 1. Setup Your Workspace

First, ensure you have a ROS 2 workspace set up. If not, follow the official ROS 2 documentation to create one.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_controller
```

#### 2. Create a Publisher Node

Let's create a simple publisher node that periodically publishes "Hello ROS 2!" messages.

Create a file `~/ros2_ws/src/my_robot_controller/my_robot_controller/simple_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2! %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 3. Create a Subscriber Node

Now, let's create a subscriber node that listens for these messages.

Create a file `~/ros2_ws/src/my_robot_controller/my_robot_controller/simple_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 4. Update `setup.py`

Modify `~/ros2_ws/src/my_robot_controller/setup.py` to include the new executables.

```python
from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simple_nodes.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_controller.simple_publisher:main',
            'simple_subscriber = my_robot_controller.simple_subscriber:main',
        ],
    },
)
```

#### 5. Build and Run

Now, build your workspace and run the nodes:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash

# In one terminal:
ros2 run my_robot_controller simple_publisher

# In another terminal:
ros2 run my_robot_controller simple_subscriber
```

You should see the subscriber node receiving messages from the publisher node.