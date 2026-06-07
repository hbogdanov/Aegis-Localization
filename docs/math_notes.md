# Math Notes

## State

x = [px, py, theta, vx, vy, omega]

## EKF

Prediction:

x_t = f(x_{t-1}, u_t) + w

Covariance:

P_t = F P F^T + Q

Update:

y = z - h(x)
S = H P H^T + R
K = P H^T S^{-1}
x = x + K y
P = (I - K H) P

## UKF

Uses sigma points to propagate uncertainty through nonlinear dynamics.

## Particle Filter

Uses weighted particles and resampling to represent arbitrary distributions.

## GTSAM Pose Graph

Nodes represent robot poses.
Edges represent odometry or loop-closure constraints.
Optimization finds the most likely pose trajectory.
