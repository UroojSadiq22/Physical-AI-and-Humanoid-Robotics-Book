--- 
title: Integrating Isaac ROS Gems
sidebar_position: 3
---

## Chapter 11: Integrating Isaac ROS Gems

### Learning Objectives

- Understand the concept of Isaac ROS Gems and their role in accelerating ROS 2 perception pipelines.
- Learn how to integrate pre-built Isaac ROS Gems into a ROS 2 workspace.
- Implement a basic perception pipeline using an Isaac ROS Gem.
- Analyze the performance benefits of GPU-accelerated ROS 2 nodes.

### Scope

This chapter focuses on the practical application of Isaac ROS. We will set up a ROS 2 workspace with Isaac ROS, integrate a specific perception Gem (e.g., for object detection or depth estimation), and demonstrate its use with simulated data.

### Key Concepts

- Isaac ROS Gems
- GPU Acceleration
- ROS 2 Packages and Nodes
- Containerization (Docker, NVIDIA Container Toolkit)
- Perception Pipelines
- Image Processing

### Practical Components

- Setting up a development environment with Isaac ROS (Docker or native).
- Integrating an Isaac ROS Gem (e.g., `isaac_ros_dope` for DOPE object pose estimation, or `isaac_ros_stereo_image_proc` for stereo depth).
- Running the Isaac ROS Gem with simulated camera data (e.g., from Isaac Sim or Gazebo).
- Visualizing the output of the Isaac ROS Gem in `rviz2`.
- Measuring the performance difference compared to CPU-only alternatives (optional, but encouraged for understanding).

### Expected Outputs

- A functional ROS 2 workspace with an integrated Isaac ROS Gem.
- A running perception pipeline that processes sensor data and outputs meaningful perception results.
- Demonstration of the GPU acceleration provided by Isaac ROS.

## Introduction to Isaac ROS Gems

**Isaac ROS Gems** are a collection of hardware-accelerated ROS 2 packages designed to bring the power of NVIDIA GPUs to common robotics tasks. These "Gems" provide highly optimized implementations for perception, navigation, and manipulation primitives, often leveraging NVIDIA's CUDA, TensorRT, and other AI acceleration libraries.

By using Isaac ROS Gems, developers can:
- Achieve significantly higher performance for perception and AI tasks.
- Reduce CPU load, freeing up resources for other computations.
- Accelerate development by using pre-optimized, production-ready components.

### Setting Up the Isaac ROS Environment

Isaac ROS Gems are typically deployed within a containerized environment (Docker) to ensure compatibility and leverage NVIDIA's GPU drivers.

#### 1. Install Docker and NVIDIA Container Toolkit

Follow the official NVIDIA Container Toolkit documentation to install Docker and the NVIDIA Container Toolkit on your Ubuntu 22.04 system. This allows Docker containers to access your GPU.

#### 2. Pull an Isaac ROS Docker Image

NVIDIA provides pre-built Docker images with ROS 2 and Isaac ROS Gems.

```bash
docker pull nvcr.io/nvidia/isaac_ros/isaac_ros_dev:humble
```
(Check NVIDIA's documentation for the latest recommended image tag.)

#### 3. Create a ROS 2 Workspace inside the Container

You'll usually mount your local ROS 2 workspace into the Docker container.

```bash
# From your ~/ros2_ws directory
docker run --runtime=nvidia -it --rm \
    -v $(pwd)/src:/workspaces/isaac_ros-dev/src \
    -v $(pwd)/install:/workspaces/isaac_ros-dev/install \
    -v $(pwd)/log:/workspaces/isaac_ros-dev/log \
    --network host \
    nvcr.io/nvidia/isaac_ros/isaac_ros_dev:humble \
    bash
```
Once inside the container, your `~/ros2_ws/src` will be mounted as `/workspaces/isaac_ros-dev/src`. You can then build your packages.

### Integrating an Isaac ROS Gem (Example: Isaac ROS Stereo Image Proc)

Let's integrate `isaac_ros_stereo_image_proc` to perform GPU-accelerated stereo depth estimation. This Gem takes two rectified stereo images and outputs a depth map and point cloud.

#### 1. Clone the Isaac ROS Repository

Inside your container (or locally, if you prefer a native setup with proper dependencies), clone the `isaac_ros_common` and `isaac_ros_image_pipeline` repositories.

```bash
cd /workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-AI-IOT/isaac_ros_common.git
git clone https://github.com/NVIDIA-AI-IOT/isaac_ros_image_pipeline.git
```

#### 2. Build the Workspace

```bash
cd /workspaces/isaac_ros-dev/
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to isaac_ros_stereo_image_proc
source install/setup.bash
```

#### 3. Prepare Input Data (Simulated Stereo Images)

You'll need a source of stereo images. This could be:
-   **Isaac Sim**: As discussed in Chapter 10, Isaac Sim can publish stereo image pairs directly to ROS 2 topics.
-   **Recorded bag file**: Play a bag file containing stereo image topics.
-   **Simulated camera node**: A simple ROS 2 node that publishes synthetic stereo images.

Let's assume you have Isaac Sim publishing to `/stereo_camera/left/image_raw` and `/stereo_camera/right/image_raw` (after rectification).

#### 4. Launch `isaac_ros_stereo_image_proc`

The `isaac_ros_stereo_image_proc` package provides launch files to easily start the stereo processing nodes.

```bash
# Ensure you are inside the Docker container and sourced your workspace
ros2 launch isaac_ros_stereo_image_proc isaac_ros_stereo_image_proc.launch.py \
    left_image_topic:=/stereo_camera/left/image_raw \
    left_camera_info_topic:=/stereo_camera/left/camera_info \
    right_image_topic:=/stereo_camera/right/image_raw \
    right_camera_info_topic:=/stereo_camera/right/camera_info \
    output_depth_pointcloud_topic:=/stereo_camera/points2 \
    # Add other parameters as needed, e.g., for rectification if not already done
```

#### 5. Visualize the Output

Launch `rviz2` (either locally or inside another terminal of the Docker container, making sure the environment is sourced).

```bash
rviz2
```
In `rviz2`:
- Add an `Image` display and subscribe to `/stereo_camera/depth` or similar depth image topic to see the depth map.
- Add a `PointCloud2` display and subscribe to `/stereo_camera/points2` to visualize the generated 3D point cloud.

### Performance Benefits

To appreciate the power of Isaac ROS, you could compare the GPU-accelerated `isaac_ros_stereo_image_proc` with its CPU-only counterpart (`stereo_image_proc` from the `image_pipeline` package).

- Measure CPU utilization, GPU utilization, and frame rate for both implementations.
- You will typically observe significantly higher frame rates and lower CPU usage with the Isaac ROS Gem, especially for high-resolution images.

### Conclusion

Integrating Isaac ROS Gems into your robotics applications provides a substantial performance boost for computationally intensive perception tasks. By leveraging NVIDIA GPUs, these Gems enable more complex and responsive robotic behaviors, bridging the gap between research and real-world deployment. As you progress in AI robotics, understanding and utilizing such hardware-accelerated libraries will be key to building cutting-edge systems.
