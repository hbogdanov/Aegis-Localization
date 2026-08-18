# Architecture

## Overview

Aegis Localization separates filter logic from ROS2 integration.

- `aegis_core`: pure C++ estimation library.
- `aegis_ros`: ROS2 nodes and launch/configuration.
- `aegis_msgs`: custom ROS messages.

## Data Flow

```text
/odom -> filter node
/imu  -> filter node
/ground_truth/pose -> trajectory logger

filter node -> /aegis/*_pose
filter node -> /aegis/*_path
trajectory logger -> CSV
evaluator -> metrics + plots
```

## Design Choices

Core filters are ROS-independent for testability. ROS nodes only handle message conversion and runtime orchestration.
