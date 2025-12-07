---
title: ROS 2 Services and Actions
sidebar_position: 4
---

## Chapter 4: ROS 2 Services and Actions

### Learning Objectives

-   Understand the difference between ROS 2 topics, services, and actions.
-   Learn how to create and use ROS 2 services for synchronous communication.
-   Learn how to create and use ROS 2 actions for long-running, goal-oriented tasks.

### Scope

This chapter focuses on the other two primary communication mechanisms in ROS 2: services and actions. We will explore their use cases, implementation details, and how they complement topics to build robust robotic applications.

### Key Concepts

-   ROS 2 Service (request-response)
-   Service client and server
-   ROS 2 Action (goal, feedback, result)
-   Action client and server
-   Synchronous vs. asynchronous communication

### Practical Components

-   Defining custom ROS 2 service and action interfaces.
-   Implementing a service server and client in Python.
-   Implementing an action server and client in Python.
-   Using `ros2 service` and `ros2 action` command-line tools.

### Expected Outputs

-   A working ROS 2 service that performs a simple calculation.
-   A working ROS 2 action that executes a long-running task with feedback.
-   Ability to interact with services and actions using command-line tools.

## Introduction to ROS 2 Services and Actions

While ROS 2 topics are excellent for streaming continuous data, some robotic tasks require a different communication pattern:
-   **Services**: For request/response interactions where a client sends a request and waits for a single response from a server. This is synchronous communication.
-   **Actions**: For long-running, goal-oriented tasks that provide periodic feedback and a final result. This is asynchronous communication, building on services and topics.

### ROS 2 Services

Services are used for synchronous communication, similar to a function call. A client sends a request, and a server processes it and sends back a response.

#### 1. Define a Custom Service

Create a file `~/ros2_ws/src/custom_interfaces/srv/AddTwoInts.srv`:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

#### 2. Update `package.xml` and `CMakeLists.txt`

Similar to custom messages, update `custom_interfaces/package.xml` with:

```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

And `custom_interfaces/CMakeLists.txt`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/AddTwoInts.srv"
)
```

#### 3. Build the Custom Service

```bash
cd ~/ros2_ws
colcon build --packages-select custom_interfaces
source install/setup.bash
```

#### 4. Implement a Service Server

Create `~/ros2_ws/src/my_robot_controller/my_robot_controller/add_two_ints_server.py`:

```python
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add two ints service server ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServer()
    rclpy.spin(add_two_ints_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 5. Implement a Service Client

Create `~/ros2_ws/src/my_robot_controller/my_robot_controller/add_two_ints_client.py`:

```python
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import AddTwoInts
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        AddTwoIntsClient.get_logger().info('Usage: ros2 run my_robot_controller add_two_ints_client <int> <int>')
        return

    add_two_ints_client = AddTwoIntsClient()
    response = add_two_ints_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    add_two_ints_client.get_logger().info(f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    add_two_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 6. Update `setup.py`

Add the new executables to `~/ros2_ws/src/my_robot_controller/setup.py`:

```python
        'console_scripts': [
            'simple_publisher = my_robot_controller.simple_publisher:main',
            'simple_subscriber = my_robot_controller.simple_subscriber:main',
            'status_publisher = my_robot_controller.status_publisher:main',
            'status_subscriber = my_robot_controller.status_subscriber:main',
            'add_two_ints_server = my_robot_controller.add_two_ints_server:main',
            'add_two_ints_client = my_robot_controller.add_two_ints_client:main',
        ],
```

#### 7. Build and Run Service

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_controller
source install/setup.bash

# In one terminal:
ros2 run my_robot_controller add_two_ints_server

# In another terminal:
ros2 run my_robot_controller add_two_ints_client 5 7
```

You should see the client sending a request and the server responding with the sum.

### ROS 2 Actions

Actions are used for long-running tasks. They provide a goal, periodic feedback, and a final result. This is useful for tasks like moving a robot to a specific pose, which might take time and require updates on progress.

#### 1. Define a Custom Action

Create a file `~/ros2_ws/src/custom_interfaces/action/Fibonacci.action`:

```
# Fibonacci.action
# Goal
int64 order
---
# Result
int64[] sequence
---
# Feedback
int64[] partial_sequence
```

#### 2. Update `package.xml` and `CMakeLists.txt`

Update `custom_interfaces/CMakeLists.txt` again:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/AddTwoInts.srv"
  "action/Fibonacci.action"
)
```

#### 3. Build the Custom Action

```bash
cd ~/ros2_ws
colcon build --packages-select custom_interfaces
source install/setup.bash
```

#### 4. Implement an Action Server

Create `~/ros2_ws/src/my_robot_controller/my_robot_controller/fibonacci_action_server.py`:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_interfaces.action import Fibonacci
import time

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Fibonacci action server ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: {0}'.format(goal_handle.request.order))
        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            sequence.append(sequence[i] + sequence[i-1])
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            time.sleep(1) # Simulate long-running task

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info('Goal succeeded, result: {0}'.format(result.sequence))
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 5. Implement an Action Client

Create `~/ros2_ws/src/my_robot_controller/my_robot_controller/fibonacci_action_client.py`:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_interfaces.action import Fibonacci
import time
import sys

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 2:
        FibonacciActionClient.get_logger().info('Usage: ros2 run my_robot_controller fibonacci_action_client <order>')
        return

    fibonacci_action_client = FibonacciActionClient()
    fibonacci_action_client.send_goal(int(sys.argv[1]))
    rclpy.spin(fibonacci_action_client)

if __name__ == '__main__':
    main()
```

#### 6. Update `setup.py`

Add the new executables to `~/ros2_ws/src/my_robot_controller/setup.py`:

```python
        'console_scripts': [
            'simple_publisher = my_robot_controller.simple_publisher:main',
            'simple_subscriber = my_robot_controller.simple_subscriber:main',
            'status_publisher = my_robot_controller.status_publisher:main',
            'status_subscriber = my_robot_controller.status_subscriber:main',
            'add_two_ints_server = my_robot_controller.add_two_ints_server:main',
            'add_two_ints_client = my_robot_controller.add_two_ints_client:main',
            'fibonacci_action_server = my_robot_controller.fibonacci_action_server:main',
            'fibonacci_action_client = my_robot_controller.fibonacci_action_client:main',
        ],
```

#### 7. Build and Run Action

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_controller
source install/setup.bash

# In one terminal:
ros2 run my_robot_controller fibonacci_action_server

# In another terminal:
ros2 run my_robot_controller fibonacci_action_client 10
```

You should see the action client receiving feedback and then the final result from the action server.
