# Data Model: Physical AI & Humanoid Robotics — Textbook

## Entities

### Textbook

-   **title**: string (e.g., "Physical AI & Humanoid Robotics")
-   **subtitle**: string (e.g., "An AI-Native Approach")
-   **modules**: has-many Module

### Module

-   **title**: string (e.g., "ROS 2 — The Robotic Nervous System")
-   **number**: integer (e.g., 1)
-   **chapters**: has-many Chapter

### Chapter

-   **title**: string (e.g., "Introduction to ROS 2")
-   **number**: integer (e.g., 1.1)
-   **learning_objectives**: list of strings
-   **scope**: string
-   **key_concepts**: list of strings
-   **practical_components**: list of strings
-   **expected_outputs**: list of strings
-   **content**: markdown
