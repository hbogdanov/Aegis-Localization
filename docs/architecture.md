# Architecture

`aegis_core` contains ROS-independent C++ implementations of the EKF, UKF, particle filter, motion model, state representation, and unit tests. `aegis_ros` owns ROS2 message conversion, runtime orchestration, synthetic sensing, timestamp-aware correction replay, and logging. `aegis_msgs` defines the filter-diagnostic message. Python packages normalize trajectories and compute metrics without changing estimator behavior.

```text
/odom, /imu, pose correction -> EKF / UKF / PF nodes -> pose and path topics
/ground_truth/pose -------------------------------------> trajectory logger
filter diagnostics -------------------------------------> trajectory logger
trajectory CSV + diagnostics -> shared evaluator -> metrics, plots, summaries
```

Delayed corrections retain bounded odometry and estimator snapshots. For replay-enabled filters, a correction is inserted at its measurement timestamp, then subsequent stored motion is reapplied and affected snapshots are rewritten. Measurements older than the retained history are rejected explicitly. The logger records ground truth, estimator trajectories, and EKF/UKF innovation diagnostics separately so performance and consistency can be evaluated independently.
