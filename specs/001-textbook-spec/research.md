# Research: Physical AI & Humanoid Robotics — Textbook

## ROS 2

**Decision**: The textbook will use ROS 2 Humble Hawksbill.

**Rationale**: ROS 2 Humble is the latest long-term support (LTS) release, ensuring stability and long-term availability of packages and tutorials. It is the standard for modern robotics development.

**Alternatives considered**: Other ROS 2 releases were considered, but Humble is the most appropriate for a textbook due to its LTS status. ROS 1 was not considered as it is a legacy system.

## Simulation Stack

**Decision**: The textbook will use both Gazebo and Unity for simulation.

**Rationale**: 
- **Gazebo**: It is the de-facto standard for robotics simulation in the ROS ecosystem. It has tight integration with ROS 2 and is excellent for simulating robot physics and sensors.
- **Unity**: Unity is a powerful game engine with high-fidelity graphics and a large asset store. It is increasingly being used for robotics simulation, especially for training AI models in photorealistic environments. The integration with NVIDIA Isaac Sim makes it a compelling choice.

**Alternatives considered**: Only using Gazebo or only using Unity. Using both provides a more comprehensive learning experience.

## Perception Pipeline

**Decision**: The textbook will primarily use Isaac ROS packages for the perception pipeline.

**Rationale**: NVIDIA Isaac ROS is a collection of high-performance ROS 2 packages for perception tasks, optimized for NVIDIA GPUs. It provides a good starting point for students to build a perception system. Generic ROS 2 nodes will be used to teach the fundamentals.

**Alternatives considered**: Using only generic ROS 2 nodes would be more fundamental but less practical for building a high-performance system.

## Vision-Language-Action (VLA) Integration

**Decision**: The VLA integration will be demonstrated using a pre-trained open-source VLA model. The focus will be on the system architecture and integration, not on training the model itself.

**Rationale**: Training a VLA model from scratch is a significant undertaking that is beyond the scope of this textbook. Using a pre-trained model allows students to focus on the integration of the VLA model with the robot's planning, navigation, and manipulation systems.

**Alternatives considered**: Training a small VLA model was considered, but it would be too complex and time-consuming for the target audience.

## Tone and Depth

**Decision**: The textbook will be aimed at a beginner–intermediate Physical AI audience.

**Rationale**: The target audience is students and developers who are new to robotics but have some programming experience. The tone will be educational and accessible, with a focus on practical skills.

**Alternatives considered**: A more advanced textbook was considered, but it would not be suitable for the target audience.
