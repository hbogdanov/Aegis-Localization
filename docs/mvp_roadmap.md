# Aegis MVP Roadmap

This roadmap intentionally stops short of a full Aegis v2 redesign.

The goal is to land the highest-return improvements first while preserving the current estimator implementations and the working synthetic benchmark.

## Phase A

Objective:

- extract the evaluation core into a reusable Python package
- define one canonical run and result schema
- create the smallest backend interface needed for synthetic plus one future public dataset backend

Non-goals:

- no generalized plugin system
- no many-backend abstraction layer
- no major ROS architecture rewrite
- no Gazebo perfection work
- no multi-dataset sweep

Completion criteria:

- `scripts/evaluate_trajectory.py` and `scripts/plot_trajectories.py` delegate to shared package code
- the repo documents one canonical run layout for future backends
- a minimal backend contract exists for synthetic and EuRoC follow-on work

## Phase B

Objective:

- implement one EuRoC sequence end to end

Requirements:

- preserve existing EKF, UKF, and PF implementations
- preserve current synthetic benchmark behavior
- reuse the shared evaluation pipeline from Phase A
- document every reduction assumption honestly
- generate the same canonical outputs as synthetic runs

Success criteria:

- one documented command
- one EuRoC sequence
- reproducible metrics
- plots
- direct comparison against the synthetic baseline

## Stop Point

After Phase B, pause for a design and evidence review before starting any research extension.

That review should answer:

- whether the EuRoC path is methodologically honest
- whether the result layout is sufficient
- whether the platform is now strong enough to support a correction-based research extension
