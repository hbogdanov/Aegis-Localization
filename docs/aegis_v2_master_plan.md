# Aegis v2 Master Plan

## Scope

This report is based on the repository state as of August 5, 2026, including:

- `README.md`
- `development_plan.md`
- `docs/architecture.md`
- `docs/future_direction.md`
- `docs/benchmark_scenarios.md`
- `docs/experiment_plan.md`
- `docs/math_notes.md`
- `AGENTS.md`
- the current ROS2, benchmark, evaluation, test, and Gazebo implementation

This is a design review, not an implementation pass. No source code changes are prescribed here.

## Executive Summary

Aegis should become a **modular robotics localization benchmarking and research platform** built around one shared evaluation core and multiple validation backends.

The repository already has the right nucleus:

- in-house EKF, UKF, and particle-filter implementations in `ros2_ws/src/aegis_core`
- ROS2 wrappers in `ros2_ws/src/aegis_ros`
- a canonical CSV evaluation path through `trajectory_logger_node.cpp` and `scripts/evaluate_trajectory.py`
- reproducible synthetic campaigns in `scripts/run_fake_benchmark_campaign.py`

The main v2 problem is that backend orchestration, dataset adaptation, result management, and experiment structure are still too implicit. The current implementation scales to synthetic and partial Gazebo use, but it will become brittle once public datasets, rosbag replay, and future hardware logs are added.

The highest-value next move is not more simulator plumbing. It is:

1. define explicit backend interfaces
2. build one public-dataset backend
3. preserve Gazebo as a secondary integration backend
4. use the platform for one real research question

---

## Part 1: Critique The Current Design

## What Will Age Well

### 1. Core estimator math is already separated from ROS2

This is the strongest design decision in the repository.

Evidence:

- `docs/architecture.md` explicitly separates `aegis_core` from `aegis_ros`
- `ros2_ws/src/aegis_core/include/aegis_core/ekf.hpp`
- `ros2_ws/src/aegis_core/include/aegis_core/ukf.hpp`
- `ros2_ws/src/aegis_core/include/aegis_core/particle_filter.hpp`
- ROS nodes call library methods rather than embedding math directly, for example `ros2_ws/src/aegis_ros/src/ekf_node.cpp`

Why it ages well:

- dataset adapters can drive the same estimators without changing estimator code
- unit tests can stay independent of ROS2
- future research extensions can live above or beside the estimator layer instead of inside ROS callbacks

### 2. The repository already has a canonical trajectory artifact format

Evidence:

- `trajectory_logger_node.cpp` writes `ekf.csv`, `ukf.csv`, `pf.csv`, and `ground_truth.csv`
- the logger standardizes the schema as `timestamp,x,y,yaw` in lines 121-129
- `scripts/evaluate_trajectory.py` assumes the same canonical columns in lines 20-53

Why it ages well:

- this is the seed of a backend-agnostic evaluation contract
- synthetic, Gazebo, rosbag, dataset, and hardware backends can all target the same normalized trajectory format

### 3. Reproducibility and metadata capture have started in the right place

Evidence:

- `scripts/run_fake_benchmark_campaign.py` records deterministic seeds in lines 157-164 and 204-213
- the same script records commit hash and UTC timestamp in lines 145-153 and 204-209
- scenario outputs are copied into per-scenario folders under `results/campaign`

Why it ages well:

- this is already close to a publication-oriented experiment record
- the missing piece is generalization across backends, not invention from scratch

### 4. Evaluation is separated from visualization

Evidence:

- `scripts/evaluate_trajectory.py` computes metrics only
- `scripts/plot_trajectories.py` is a separate plotting script

Why it ages well:

- this separation makes it easier to add new plots, tables, and reports without entangling metric computation

### 5. The synthetic backend is already useful as a control environment

Evidence:

- `fake_sensor_publisher_node.cpp` generates repeatable trajectories, noise, and dropout
- `fake_benchmark.launch.py` parameterizes noise, seeds, duration, and estimator selection
- `docs/benchmark_scenarios.md` already defines named scenarios

Why it ages well:

- synthetic remains the best place for controlled ablations, stress tests, and regression protection even after public datasets are added

## What Will Become Liabilities

### 1. Backend orchestration is hard-coded into ad hoc scripts

Evidence:

- `scripts/run_fake_benchmark_campaign.py` directly contains WSL launching, cleanup, scenario definitions, metrics invocation, metadata, and file copying
- `scripts/run_gazebo_validation.py` repeats similar concerns for Gazebo with different paths and process cleanup

Why this becomes a liability:

- every new backend will duplicate launch, cleanup, artifact validation, metric calls, and output layout
- dataset backends will likely create a third orchestration script instead of fitting a reusable pattern

### 2. ROS topics are treated as the backend API

Evidence:

- `ekf_node.cpp`, `ukf_node.cpp`, and `particle_filter_node.cpp` subscribe directly to `/odom` and `/imu`
- `trajectory_logger_node.cpp` subscribes directly to `/ground_truth/pose` and estimator pose topics
- `gazebo_ground_truth_bridge_node.cpp` exists only to remap Gazebo truth into the expected topic

Why this becomes a liability:

- public datasets rarely arrive in the exact `/odom`, `/imu`, `/ground_truth/pose` shape
- future backends will need more and more one-off bridge nodes unless an explicit adapter boundary is introduced

### 3. Result organization is only partially backend-aware

Evidence:

- synthetic uses `results/metrics` and `results/campaign`
- Gazebo uses `results/gazebo_metrics` and `results/gazebo_validation`
- `scripts/evaluate_trajectory.py` still defaults to writing JSON to `results/metrics`

Why this becomes a liability:

- dataset backends will either overload synthetic directories or proliferate new custom directories
- comparison across backends becomes harder if outputs are not organized by a single predictable schema

### 4. Environment assumptions are still embedded in core benchmark scripts

Evidence:

- `scripts/run_fake_benchmark_campaign.py` hard-codes `wsl.exe -d Ubuntu-22.04` in lines 63-69
- the same script converts Windows paths into WSL paths in lines 72-76
- `scripts/run_gazebo_validation.py` repeats the same WSL assumptions

Why this becomes a liability:

- dataset evaluation should not depend on a specific Windows plus WSL deployment pattern
- this hurts portability, CI, and long-term maintainability

### 5. The current metrics layer is too narrow for publication-level evaluation

Evidence:

- `scripts/evaluate_trajectory.py` only computes sample count, ATE RMSE, final drift, and yaw RMSE in lines 88-103
- `docs/experiment_plan.md` already names RPE and failure recovery behavior, but the implementation does not yet compute them

Why this becomes a liability:

- public dataset support will immediately raise expectations for RPE, distance-normalized drift, failure counts, and robustness summaries
- serious reports need richer metrics and stronger failure analysis

## Abstractions That Already Exist

- estimator library boundary: `aegis_core` versus `aegis_ros`
- canonical trajectory artifact: CSV with `timestamp,x,y,yaw`
- estimator family abstraction: EKF, UKF, PF sharing similar ROS wrapper behavior
- run metadata concept: seed, duration, commit hash, timestamp
- benchmark scenario concept: named synthetic scenarios in `run_fake_benchmark_campaign.py`

## Abstractions That Are Missing

- explicit backend interface
- explicit dataset adapter interface
- explicit run manifest or experiment spec
- shared result schema across all backends
- reusable metric registry
- reusable plotting/report generation pipeline
- a normalized sensor-stream representation above raw ROS topics and below dataset-specific formats

## Modules That Are Too Tightly Coupled

### 1. Backend execution and experiment bookkeeping

Evidence:

- `scripts/run_fake_benchmark_campaign.py` mixes launch, cleanup, evaluation, metadata, and result copying
- `scripts/run_gazebo_validation.py` does the same for Gazebo

These should be separated into:

- backend runner
- result validator
- metric executor
- report aggregator

### 2. Logging and benchmark semantics

Evidence:

- `trajectory_logger_node.cpp` both discovers repository-relative output paths and defines the canonical benchmark artifact names

This couples:

- runtime ROS logging
- repo layout assumptions
- evaluation artifact schema

### 3. Estimator wrappers and sensor contract

Evidence:

- the filter nodes directly assume `/odom` and `/imu` as the live measurement API

That is fine for ROS runtime, but too rigid for multi-backend benchmarking. A dataset adapter should produce a normalized input contract.

## Pieces That Should Become Reusable Libraries

### 1. Evaluation core

Current source:

- `scripts/evaluate_trajectory.py`
- parts of `scripts/run_fake_benchmark_campaign.py`

Should become:

- `python/aegis_eval/metrics.py`
- `python/aegis_eval/io.py`
- `python/aegis_eval/summary.py`
- `python/aegis_eval/plots.py`

### 2. Experiment orchestration layer

Current source:

- `scripts/run_fake_benchmark_campaign.py`
- `scripts/run_gazebo_validation.py`

Should become:

- backend runners plus a shared experiment driver

### 3. Sensor adapter layer

Current source:

- currently implicit in ROS topics and ad hoc bridge nodes

Should become:

- reusable adapters from dataset or rosbag formats into a normalized Aegis measurement stream

---

## Part 2: Design Aegis v2

## Design Goal

One evaluation framework should support:

- Synthetic benchmarks
- Public datasets
- ROS bag replay
- Gazebo
- Future hardware logs

The architecture should minimize duplicated logic by separating:

- estimator math
- runtime adapters
- backend orchestration
- metric computation
- report generation

## Proposed Directory Layout

```text
docs/
  aegis_v2_master_plan.md
  architecture_v2.md
  methodology.md
  benchmark_report.md
  dataset_notes/
    euroc.md
    gazebo.md
    synthetic.md

ros2_ws/
  src/
    aegis_core/
    aegis_msgs/
    aegis_ros/
    aegis_adapters/
      include/aegis_adapters/
      src/

python/
  aegis_eval/
    __init__.py
    io.py
    metrics.py
    metadata.py
    plots.py
    reports.py
    schemas.py
  aegis_bench/
    __init__.py
    backends/
      base.py
      synthetic.py
      gazebo.py
      rosbag.py
      euroc.py
    runner.py
    manifests.py
    paths.py

benchmarks/
  synthetic/
    scenarios/
      low_noise.yaml
      high_noise_dropout.yaml
      dead_reckoning.yaml
  gazebo/
    scenarios/
      tb3_circle.yaml
  datasets/
    euroc/
      mh_01.yaml
      v1_01.yaml
  rosbags/
    sample_replay.yaml

configs/
  estimators/
    ekf.yaml
    ukf.yaml
    pf.yaml
  evaluation/
    default_metrics.yaml
    publication_metrics.yaml

results/
  synthetic/
  gazebo/
  datasets/
  rosbags/
  hardware/
```

## Package Boundaries

### `aegis_core`

Responsibility:

- estimator math
- motion model
- uncertainty propagation
- future correction hooks

Must not depend on ROS2.

### `aegis_ros`

Responsibility:

- ROS2 nodes for estimator execution
- launch files
- runtime publishers/subscribers

Should remain a runtime wrapper layer, not an experiment-management layer.

### `aegis_adapters`

Responsibility:

- ROS-side adapters and bridges
- dataset-to-topic playback helpers
- rosbag replay helpers
- ground-truth bridge nodes

This keeps adapter logic out of estimator wrappers.

### `python/aegis_eval`

Responsibility:

- loading canonical outputs
- metrics
- aggregation
- plotting
- table generation
- report artifacts

### `python/aegis_bench`

Responsibility:

- experiment manifests
- backend execution
- backend-specific staging
- standardized result directory creation
- metadata capture

## Backend Interface

Each backend should implement the same conceptual interface:

```text
prepare(manifest) -> prepared run context
execute(context) -> raw artifacts
normalize(raw artifacts) -> canonical trajectory outputs
validate(canonical outputs) -> pass/fail with diagnostics
summarize(canonical outputs) -> metrics + metadata bundle
```

Practical backend responsibilities:

- Synthetic backend: launch fake publisher benchmark with exact seeds and scenario args
- Gazebo backend: launch simulation, drive command profile, capture truth and estimator outputs
- Rosbag backend: replay bag, remap topics, capture outputs
- Dataset backend: convert source data into normalized ROS or offline trajectory inputs
- Hardware backend: replay recorded robot logs through the same normalized path

## Adapter Pattern

Use a two-stage adapter model.

### Stage 1: source adapter

Transforms a backend-native source into an Aegis-normalized measurement contract.

Examples:

- EuRoC files to normalized time series
- rosbag topics to normalized measurement topics
- Gazebo truth odometry to normalized ground-truth pose

### Stage 2: execution adapter

Feeds normalized measurements through one of two execution paths:

- online ROS execution path
- offline evaluator path when full ROS replay is unnecessary

This avoids forcing every backend through the exact same launch pattern while preserving identical outputs.

## Normalized Interfaces

### Normalized estimator input contract

At minimum:

- timestamp
- planar pose observation when available
- planar velocity observation when available
- yaw or yaw-rate observation when available
- ground truth when available
- backend metadata and frame assumptions

### Normalized estimator output contract

Per estimator:

- `trajectory.csv`
- optional covariance or confidence export
- optional diagnostics stream

### Canonical run output contract

Per run:

- `ground_truth.csv`
- `ekf.csv`
- `ukf.csv`
- `pf.csv`
- `metrics/`
- `plots/`
- `metadata.json`
- `manifest.json`
- `stdout.log`
- `environment.json`

## Evaluation Pipeline

The full pipeline should be:

1. load benchmark manifest
2. resolve estimator configs
3. stage backend inputs
4. run backend
5. export canonical trajectories
6. validate sample counts, timestamps, and frame assumptions
7. compute primary metrics
8. compute robustness and failure metrics when applicable
9. generate plots and tables
10. write run metadata and reproducibility bundle

## Configuration System

Use declarative manifests rather than script-local constants.

Each benchmark manifest should specify:

- backend
- scenario or dataset
- estimator list
- estimator config paths
- duration or sequence range
- seed
- expected topics or signals
- metrics profile
- plot profile
- notes and assumptions

This replaces hard-coded scenario dictionaries like the current `SCENARIOS` list in `run_fake_benchmark_campaign.py`.

## Result Organization

Use one stable layout:

```text
results/
  <backend>/
    <benchmark_name>/
      <run_id>/
        manifest.json
        metadata.json
        raw/
        normalized/
          ground_truth.csv
          ekf.csv
          ukf.csv
          pf.csv
        metrics/
          ekf_metrics.json
          ukf_metrics.json
          pf_metrics.json
          summary.json
        plots/
          trajectories.png
          position_error.png
          yaw_error.png
        logs/
```

Benefits:

- every backend looks the same after normalization
- publication tables can aggregate across backends automatically
- archived evidence can be tied to exact manifests

## Benchmark Organization

Separate benchmark definitions from execution scripts.

Benchmark units:

- synthetic scenario
- Gazebo scenario
- dataset sequence
- rosbag replay case
- hardware session

Each benchmark should have:

- a manifest
- expected outputs
- documented assumptions
- optional baseline reference metrics

## Metadata Organization

Every run should record:

- backend type
- benchmark name
- run id
- estimator versions and configs
- commit hash
- ROS and Python environment summary
- seed
- frame conventions
- data provenance
- command or manifest used
- timing summary
- truth source type: synthetic, simulation, dataset, or hardware

## Plotting Pipeline

Have plot generation consume only canonical outputs and metric summaries.

Required standard plots:

- XY trajectory overlay
- position error over time
- yaw error over time
- histogram or CDF of translational error
- per-estimator summary bar charts

Optional backend-specific plots:

- dropout intervals
- relocalization recovery intervals
- covariance traces

## Experiment Organization

Organize experiments by question, not by backend alone.

Example experiment families:

- baseline estimator comparison
- robustness under noise and dropout
- cross-backend generalization
- correction-method comparison
- delayed-measurement sensitivity

This is important because a research platform should answer questions, not just run demos.

---

## Part 3: Public Dataset Strategy

## Candidate Comparison

| Dataset | Engineering Effort | Compatibility With Current Estimators | Required Adapters | Required Assumptions | Likely Metrics | Research Credibility | Hiring Value | Graduate-School Value |
|---|---|---|---|---|---|---|---|---|
| EuRoC MAV | Moderate | Moderate | timestamp association, frame normalization, planarization or selective measurement extraction, IMU adapter | 3D data must be reduced or wrapped into Aegis’s current planar contract unless estimators are extended | ATE, RPE, drift, timing, robustness | High | High | High |
| KITTI | Moderate to high | Moderate | vehicle odometry adapter, IMU and pose alignment, frame conversion | car-like motion is a narrower fit than general robotics | ATE, RPE, drift per distance | High | High | High |
| TUM RGB-D | Moderate | Low to moderate | RGB-D or pose stream adapter, timestamp association | current project is not vision-first yet | ATE, RPE, trajectory alignment | Moderate to high | Moderate | Moderate to high |
| Hilti SLAM | High | Low to moderate | multi-sensor parsing, complex frame handling, likely more assumptions than current repo supports | strong sensor and platform assumptions | ATE, drift, robustness | High | Moderate | High |
| Newer College | High | Low to moderate | dataset parsing, frame normalization, likely substantial adaptation | richer platform than current estimators naturally fit | ATE, drift, robustness | High | Moderate | High |
| ROS-compatible public bags | Low to moderate | High | topic remap plus truth adapter | data quality varies by source | ATE, yaw RMSE, drift, update rate, failures | Moderate to high | High | High |

## Detailed Notes

### EuRoC MAV

Strengths:

- recognized benchmark
- strong research signal
- real IMU data
- good publication credibility

Challenges:

- Aegis is currently planar and odom-centric
- EuRoC is naturally a 6-DoF dataset
- the first implementation will need careful scoping to avoid pretending the estimator solves more than it does

### KITTI

Strengths:

- strong autonomy hiring signal
- good reproducibility

Challenges:

- makes Aegis feel more AV-specific than robotics-general
- not as natural a fit if the long-term project is indoor/mobile-robot localization plus correction research

### TUM RGB-D

Strengths:

- perception relevance
- strong if visual correction becomes central

Challenges:

- weak fit for the current non-visual estimator stack
- better as a second dataset after a more natural localization backend exists

### Hilti SLAM and Newer College

Strengths:

- strong research credibility
- rich sensors and realistic conditions

Challenges:

- too heavy for the first dataset integration
- high risk of infrastructure and assumption debt

### ROS-compatible public bags

Strengths:

- most natural bridge from the current ROS2 architecture
- preserves the existing runtime path
- likely the best cost-to-value ratio

Challenges:

- credibility depends on the dataset choice and documentation quality
- some public bags have inconsistent truth quality

## Recommendation: Implement EuRoC MAV First

I recommend **EuRoC MAV** as the first named public dataset, with a carefully scoped planar evaluation mode.

Why EuRoC over a generic ROS bag:

- stronger name recognition for faculty and research-oriented reviewers
- higher long-term ceiling for publication-style evaluation
- clearer signal that Aegis can extend beyond self-authored synthetic data

Why EuRoC despite the adaptation work:

- the engineering work is meaningful rather than cosmetic
- the adaptation itself tells a strong story about frame discipline, timestamp handling, and experimental honesty
- it creates the right pressure to build the modular backend interface correctly

Required discipline:

- explicitly document that current estimators are planar and describe the reduction assumptions
- do not market the result as a full 6-DoF state-estimation solution

If a lower-risk bridge is desired before EuRoC, a ROS-bag backend can be built first as an internal stepping stone, but the first flagship external dataset should still be EuRoC.

---

## Part 4: Research Roadmap

## Direction 1: Visual Or Landmark-Assisted Correction

### Research question

How much can intermittent exteroceptive correction reduce drift and improve recovery under noise and dropout compared with pure odometry plus IMU fusion?

### Novelty relative to the current repo

The repository currently compares classical recursive estimators. It does not yet study correction from external observations.

### Implementation effort

Moderate to high.

### Expected engineering value

- introduces a clean correction interface
- improves backend relevance for vision-aware robotics
- enables richer dataset choices later

### Expected research value

High.

### Expected interview value

High, because it turns the project into a system that answers a real robotics question instead of only implementing known filters.

### Measurable experiments

- correction frequency sweep
- dropout versus correction quality
- drift recovery after long dead-reckoning intervals
- sensitivity to corrupted or delayed landmark updates

### Evaluation methodology

- compare EKF, UKF, PF with and without correction
- run on synthetic plus one public dataset
- report ATE, RPE, recovery time, and failure rate

## Direction 2: Adaptive Covariance Estimation

### Research question

Can online adaptation of process and measurement noise improve robustness across changing noise regimes without hand tuning?

### Novelty relative to the current repo

The current repo uses fixed noise settings through configuration.

### Implementation effort

Moderate.

### Expected engineering value

- improves estimator configurability
- creates a more realistic deployment story

### Expected research value

Moderate to high.

### Expected interview value

Good, especially for estimation-heavy roles.

### Measurable experiments

- noise-regime shifts
- mismatch between assumed and actual sensor noise
- dropout plus noise interaction

### Evaluation methodology

- ablate static versus adaptive tuning
- report performance variance across scenarios, not just mean metrics

## Direction 3: Delayed And Asynchronous Measurement Fusion

### Research question

How robust are the estimators to delayed, missing, or asynchronously sampled measurements, and what correction strategies best mitigate this?

### Novelty relative to the current repo

Current synthetic dropout is a start, but delayed and asynchronous fusion is not a first-class research axis yet.

### Implementation effort

Moderate.

### Expected engineering value

- closer to real robotics systems
- strong systems-design story

### Expected research value

Moderate to high.

### Expected interview value

High for robotics software and autonomy roles.

### Measurable experiments

- controlled delay injection
- frequency mismatch studies
- stale measurement rejection versus acceptance strategies

### Evaluation methodology

- measure failure rate, drift growth, and recovery under timing distortions

## Primary Recommendation

The primary direction should be **visual or landmark-assisted correction**.

Why it wins:

- strongest research narrative
- best bridge to robotics perception relevance
- easiest to explain to faculty and hiring managers
- naturally leverages the benchmarking platform rather than replacing it

---

## Part 5: Publication-Level Evaluation

## Objective

Evaluation should look like a serious robotics technical report, not a GitHub demo.

## Baselines

Primary baselines:

- EKF
- UKF
- PF

Future baseline classes:

- dead-reckoning only
- best current Aegis estimator without correction
- corrected estimator variant

Optional later baselines:

- ROS-standard localization baseline if used carefully and honestly
- simple oracle-style intermittent correction upper bound

## Datasets And Backends

Minimum serious evaluation stack:

- synthetic benchmark for controlled ablations
- one public dataset backend such as EuRoC
- optional Gazebo backend as integration evidence, not flagship evidence

## Metrics

Core metrics:

- ATE RMSE
- RPE
- final drift
- drift per traveled distance
- yaw RMSE
- update rate
- valid sample count

Robustness metrics:

- failure rate
- divergence count
- recovery time after dropout
- time above error thresholds

Research metrics for correction work:

- correction acceptance rate
- correction-induced improvement
- false-correction damage cases

## Ablation Studies

Required ablations:

- estimator family: EKF versus UKF versus PF
- with and without pose update
- low noise versus high noise versus dropout
- seed sensitivity
- dataset sequence variation

If correction is added:

- correction interval
- correction noise quality
- delayed correction
- correction availability rate

## Robustness Experiments

Synthetic robustness:

- sensor dropout sweep
- noise sweep
- zero-motion sanity
- angle-wrap edge cases
- asynchronous sensor-rate mismatch

Dataset robustness:

- sequence difficulty variation
- degraded input modes
- truth-association sensitivity

## Failure Analysis

Every serious report should include explicit failures:

- sequences where an estimator diverges
- cases where PF particle impoverishment appears
- cases where UKF or EKF become unstable under wrong assumptions
- cases where correction hurts rather than helps

The project becomes more credible when failure is characterized, not hidden.

## Reproducibility Requirements

- named manifests for every benchmark
- fixed seeds where applicable
- exact estimator config snapshots
- exact dataset sequence names
- commit hash in every run
- generated summary tables from result artifacts, not hand-written values
- documented environment assumptions

## Benchmark Tables

Minimum tables:

- per-backend estimator comparison
- cross-backend summary
- robustness sweep summary
- ablation summary for the primary research extension

## Figures And Plots

Required:

- trajectory overlays
- translational error over time
- yaw error over time
- box plot or CDF across runs
- drift versus dropout or noise level

Useful later:

- recovery after correction event
- covariance trace or uncertainty proxy
- failure-case visualizations

---

## Part 6: Technical Report Plan

## 1. Architecture Report

Audience:

- robotics software recruiters
- faculty reviewers
- collaborators

Approximate length:

- 6 to 8 pages

Figures:

- system architecture diagram
- backend adapter diagram
- evaluation pipeline diagram

Tables:

- package responsibilities
- data contracts

Code references:

- `aegis_core`
- `aegis_ros`
- evaluation and backend packages

## 2. Benchmark Report

Audience:

- hiring managers
- robotics labs
- portfolio reviewers

Approximate length:

- 8 to 12 pages

Figures:

- benchmark workflow
- sample trajectories
- error plots

Tables:

- primary metrics per backend
- robustness summaries

Code references:

- benchmark manifests
- metric pipeline

## 3. Experimental Methodology

Audience:

- faculty
- research labs

Approximate length:

- 4 to 6 pages

Figures:

- dataset/backend setup diagrams

Tables:

- sequence list
- metric definitions
- ablation matrix

Code references:

- manifest schema
- evaluation scripts and library functions

## 4. Implementation Notes

Audience:

- engineers
- future maintainers

Approximate length:

- 4 to 5 pages

Figures:

- package dependency graph

Tables:

- configuration keys
- file layout

Code references:

- launch files
- adapter modules
- logger and normalization code

## 5. Mathematical Derivations

Audience:

- faculty
- technically deep interviewers

Approximate length:

- 5 to 8 pages

Figures:

- state and measurement model diagrams

Tables:

- notation table
- model assumptions

Code references:

- `aegis_core` estimators
- `docs/math_notes.md`

## 6. Failure Analysis

Audience:

- faculty
- research reviewers
- strong interviewers

Approximate length:

- 3 to 5 pages

Figures:

- failure trajectories
- divergence examples

Tables:

- failure categories
- root-cause summary

Code references:

- failure case manifests
- robustness experiment outputs

## 7. Lessons Learned

Audience:

- general portfolio readers
- recruiters

Approximate length:

- 2 to 3 pages

Figures:

- timeline or architecture evolution graphic

Tables:

- design decisions and outcomes

Code references:

- key commits or milestone diffs

---

## Part 7: Gap Analysis

## Ranked By Return On Investment

| Rank | Missing Capability | Why It Matters | Difficulty | Engineering Value | Research Value | Hiring Value | Graduate-School Value |
|---|---|---|---|---|---|---|---|
| 1 | Modular backend interface | prevents future duplication and fragility | Moderate | Very high | High | High | High |
| 2 | One public dataset backend | provides external credibility | Moderate to high | High | Very high | High | Very high |
| 3 | Richer evaluation metrics | needed for serious technical reporting | Moderate | High | High | High | High |
| 4 | Unified result schema | makes experiments maintainable and comparable | Moderate | High | High | Moderate | High |
| 5 | Publication-style reporting pipeline | converts engineering into evidence | Moderate | Moderate | High | High | High |
| 6 | Correction research extension | creates a compelling intellectual contribution | High | High | Very high | High | Very high |
| 7 | Timing and delay robustness framework | improves realism and systems depth | Moderate | High | High | High | High |
| 8 | Stable Gazebo scoring | useful integration evidence, but secondary | Moderate to high | Moderate | Moderate | Moderate | Moderate |

## Detailed Gap Notes

### Modular backend interface

Why it matters:

- current synthetic and Gazebo scripts duplicate orchestration patterns
- this is the root architectural blocker for v2 growth

### Public dataset backend

Why it matters:

- strongest credibility jump available
- breaks the self-generated-data ceiling

### Richer evaluation metrics

Why it matters:

- current metrics are too shallow for research-style claims
- needed before serious dataset comparison

### Unified result schema

Why it matters:

- current split between `results/metrics`, `results/campaign`, `results/gazebo_metrics`, and `results/gazebo_validation` will not scale cleanly

### Reporting pipeline

Why it matters:

- the project needs auto-generated evidence, not manually curated summaries

### Correction extension

Why it matters:

- this is what upgrades Aegis from benchmark platform to research platform

### Timing and delay robustness

Why it matters:

- real robots fail on timing issues, not just Gaussian noise

### Stable Gazebo scoring

Why it matters:

- useful for demos and integration validation
- lower ROI than dataset support because of infrastructure risk

---

## Part 8: Master Development Plan

## P0: Credibility Blockers

### Milestone P0.1: Freeze Honest Project Scope

Objective:

- align README, docs, and reports around simulation-backed plus dataset-targeted claims only

Affected modules:

- `README.md`
- `docs/`

Estimated effort:

- 4 hours

Dependencies:

- none

Completion criteria:

- no text overstates Gazebo or hardware validation
- current supported backends and limitations are explicit

Expected impact:

- protects credibility immediately

### Milestone P0.2: Define Canonical Run Schema

Objective:

- standardize normalized outputs, metadata, and directory layout for every backend

Affected modules:

- new `python/aegis_eval/schemas.py`
- `scripts/` replacement work
- docs

Estimated effort:

- 8 hours

Dependencies:

- P0.1

Completion criteria:

- written schema for trajectories, metadata, metrics, and results tree
- one example manifest and one example run tree documented

Expected impact:

- removes ambiguity before more backend code is written

## P1: Architecture Improvements

### Milestone P1.1: Extract Evaluation Core

Objective:

- move CSV loading, alignment, metrics, and summaries into a reusable Python package

Affected modules:

- `scripts/evaluate_trajectory.py`
- new `python/aegis_eval/*`

Estimated effort:

- 12 hours

Dependencies:

- P0.2

Completion criteria:

- script becomes a thin CLI wrapper over library functions
- unit tests cover core metric logic

Expected impact:

- prevents every backend from re-implementing evaluation behavior

### Milestone P1.2: Introduce Backend Runner Interface

Objective:

- replace ad hoc backend scripts with a shared runner and backend classes

Affected modules:

- `scripts/run_fake_benchmark_campaign.py`
- `scripts/run_gazebo_validation.py`
- new `python/aegis_bench/backends/*`

Estimated effort:

- 16 hours

Dependencies:

- P1.1

Completion criteria:

- synthetic and Gazebo both run through the same driver
- per-backend specifics are isolated to backend modules

Expected impact:

- the repo becomes structurally ready for public datasets

### Milestone P1.3: Introduce Explicit Benchmark Manifests

Objective:

- move scenario definitions out of script-local constants into versioned manifest files

Affected modules:

- `benchmarks/`
- backend runner

Estimated effort:

- 10 hours

Dependencies:

- P1.2

Completion criteria:

- synthetic benchmark scenarios and one Gazebo scenario are manifest-driven

Expected impact:

- reproducibility improves and report generation gets simpler

## P2: Dataset Support

### Milestone P2.1: Build Normalized Dataset Adapter Contract

Objective:

- define how external datasets are converted into Aegis-normalized signals

Affected modules:

- `aegis_adapters`
- `python/aegis_bench/backends`
- docs

Estimated effort:

- 10 hours

Dependencies:

- P1.2

Completion criteria:

- documented adapter interface
- one test fixture with synthetic normalized data

Expected impact:

- de-risks the first public dataset integration

### Milestone P2.2: Implement EuRoC Backend

Objective:

- run at least one EuRoC sequence through the shared evaluation framework

Affected modules:

- dataset backend code
- manifests
- docs

Estimated effort:

- 28 hours

Dependencies:

- P2.1

Completion criteria:

- exact documented command for one EuRoC sequence
- canonical outputs generated
- metrics and plots produced
- reduction assumptions documented clearly

Expected impact:

- biggest credibility jump in the roadmap

### Milestone P2.3: Cross-Backend Comparison Table

Objective:

- generate one summary table comparing synthetic and EuRoC results

Affected modules:

- evaluation/report pipeline

Estimated effort:

- 8 hours

Dependencies:

- P2.2

Completion criteria:

- report-ready table generated from result artifacts

Expected impact:

- demonstrates platform coherence rather than isolated backend demos

## P3: Research Extensions

### Milestone P3.1: Correction Interface

Objective:

- create a clean estimator-side hook for intermittent correction updates

Affected modules:

- `aegis_core`
- `aegis_ros`
- adapter docs

Estimated effort:

- 14 hours

Dependencies:

- P2.2

Completion criteria:

- design and minimal implementation for external correction updates

Expected impact:

- establishes the platform’s research direction

### Milestone P3.2: Visual Or Landmark Correction Prototype

Objective:

- implement one correction mechanism and compare it to the baseline estimators

Affected modules:

- correction module
- benchmark manifests
- evaluation pipeline

Estimated effort:

- 24 hours

Dependencies:

- P3.1

Completion criteria:

- corrected estimator variant runs on synthetic and one public dataset
- measurable benefit or documented failure cases

Expected impact:

- highest research and interview upside in the entire roadmap

### Milestone P3.3: Robustness Study

Objective:

- evaluate correction under dropout, delay, and noise

Affected modules:

- synthetic manifests
- dataset manifests
- evaluation pipeline

Estimated effort:

- 12 hours

Dependencies:

- P3.2

Completion criteria:

- ablation plots and summary tables for robustness

Expected impact:

- turns the extension into a real experiment, not just a feature

## P4: Documentation

### Milestone P4.1: Architecture v2 Report

Objective:

- document the modular system design cleanly

Affected modules:

- `docs/architecture_v2.md`

Estimated effort:

- 8 hours

Dependencies:

- P1.3

Completion criteria:

- package boundaries, data flow, manifests, and result schema documented

Expected impact:

- helps both maintainability and interview storytelling

### Milestone P4.2: Benchmark Methodology Report

Objective:

- formalize evaluation methodology, metrics, and experimental discipline

Affected modules:

- `docs/methodology.md`

Estimated effort:

- 8 hours

Dependencies:

- P2.3

Completion criteria:

- datasets, metrics, and ablations documented precisely

Expected impact:

- strong graduate-school signal

### Milestone P4.3: Benchmark Report

Objective:

- publish a polished report with figures, tables, and takeaways

Affected modules:

- `docs/benchmark_report.md`
- plots and tables pipeline

Estimated effort:

- 12 hours

Dependencies:

- P3.3

Completion criteria:

- one report that a faculty reviewer could read independently of the repo

Expected impact:

- converts engineering work into visible evidence

## P5: Optional Future Work

### Milestone P5.1: Gazebo Stabilization

Objective:

- finish Gazebo scoring only if it can be made low-friction within the new backend architecture

Affected modules:

- Gazebo backend
- adapters
- manifests

Estimated effort:

- 10 to 20 hours

Dependencies:

- P1.2

Completion criteria:

- stable truth logging and evaluation through the shared pipeline
- otherwise explicitly kept as integration-only

Expected impact:

- useful demo value, secondary research value

### Milestone P5.2: Additional Datasets

Objective:

- add KITTI, TUM RGB-D, or a strong ROS-bag dataset after the first external backend succeeds

Affected modules:

- dataset adapters
- manifests

Estimated effort:

- 20 to 30 hours each

Dependencies:

- P2.2

Completion criteria:

- one added backend with clean documentation and comparable outputs

Expected impact:

- broadens the platform and improves generality

---

## Part 9: Final Recommendation

### 1. What should Aegis become in one sentence?

Aegis should become a **modular robotics localization benchmarking and research platform that evaluates in-house estimators across synthetic, simulated, and public-data backends with publication-quality evidence**.

### 2. What single capability would most impress robotics faculty?

A clean public-dataset evaluation backend, paired with one research-grade correction experiment and honest methodology.

### 3. What single capability would most impress robotics hiring managers?

A reproducible multi-backend benchmark pipeline that runs the same EKF, UKF, and PF stack across controlled synthetic and external real-data evaluation with strong metrics and failure analysis.

### 4. What work should never be done because it adds complexity without meaningful value?

Do not spend large amounts of time building simulator-specific infrastructure, extra launch permutations, or flashy but weak demos that do not improve external validation, research depth, or reproducibility.

### 5. If limited to 150 additional engineering hours, exactly how should they be spent?

Recommended allocation:

- 20 hours: canonical run schema, manifests, and backend interface
- 28 hours: extract reusable evaluation core and report pipeline
- 32 hours: implement EuRoC backend with honest planar assumptions
- 12 hours: cross-backend tables, plots, and reproducibility bundle
- 18 hours: correction interface design and baseline implementation
- 22 hours: visual or landmark-assisted correction prototype
- 10 hours: robustness ablations on synthetic plus dataset runs
- 8 hours: architecture and methodology documentation

This budget prioritizes platform leverage, external credibility, and one real research contribution over roadmap inertia.

## Bottom Line

The current repository is already past the point of being just a ROS2 filter package. Its strongest future is not to become larger in every direction. Its strongest future is to become sharper:

- one shared backend architecture
- one serious external dataset
- one compelling research extension
- one report-quality evaluation story

That path maximizes engineering value, interview depth, research credibility, and graduate-school competitiveness.
