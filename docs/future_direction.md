# Future Direction Review

> **Implementation update (2026-08-19):** This document records the design decision that led to the current direction. Since the review, the repository has completed the single-sequence EuRoC planar proxy, repeated-seed synthetic evidence, EKF/UKF consistency analysis, and the Phase 4 gating study. Phase 5 now has intermittent correction infrastructure and verified EKF/UKF delayed-replay correctness. The active implementation target is the small naive-late-fusion versus timestamp-aware-replay comparison, followed by one compact degradation campaign. See `development_plan.md` for the current execution state.

## Context

This review is based on:

- `development_plan.md`
- `docs/architecture.md`
- `README.md`
- the current synthetic benchmark pipeline
- the current Gazebo validation and scoring implementation
- the current repository structure and code boundaries

The purpose of this review is not to maximize roadmap completion. It is to maximize the value of Aegis as a flagship project for:

- robotics perception internships
- robotics software roles
- MS robotics admissions
- robotics research labs

## Part 1: Project Identity

### Recommendation

The strongest primary identity for Aegis is:

**robotics localization benchmarking and research platform**

### Why this is the strongest identity

The repository is no longer just a filter implementation package.

It already contains:

- in-house EKF, UKF, and PF implementations
- a repeatable benchmark runner
- shared evaluation scripts
- trajectory logging
- metrics generation
- reproducibility metadata
- growing regression coverage

That makes its center of gravity broader than a plain ROS2 localization package and more ambitious than a simple estimator comparison demo.

### Why the alternatives are weaker

`localization algorithm library`

- Too narrow.
- `aegis_core` does fit this label, but the repo already includes orchestration, experiments, evidence generation, and evaluation logic.
- This framing undersells the project.

`ROS2 localization package`

- Technically true, but too implementation-oriented.
- This sounds like deployment plumbing rather than a serious evaluation or research asset.
- It is weaker for graduate admissions and weaker for research-lab positioning.

`estimator comparison framework`

- Close, but still too narrow.
- It captures the synthetic benchmark story, but not the longer-term value of public datasets, future hardware, or exteroceptive correction research.

`localization benchmarking framework`

- Strong, and probably the second-best label.
- The reason I prefer “benchmarking and research platform” is that Aegis should not stop at reporting metrics.
- The highest-value version of this project is one where benchmarking is the foundation for answering research questions about robustness, correction, uncertainty, and multimodal localization.

### Final identity statement

Aegis should become a **modular robotics localization benchmarking and research platform** that:

- implements classical estimators in-house
- evaluates them reproducibly across multiple backends
- supports future exteroceptive correction research
- produces evidence that is both engineering-relevant and research-credible

## Part 2: Evaluate Phase 5

### Current Gazebo milestone assessment

The current Gazebo milestone was a reasonable intermediate target, but it is **no longer obviously the highest-value next investment**.

### Why

#### 1. Remaining engineering effort is unpredictable

The current Gazebo path is no longer a straightforward “finish the last 10%” task.

The failure mode is no longer just:

- logger path bug
- stale directory bug
- missing CSV write

It has become simulator infrastructure work involving:

- Gazebo startup stability
- spawn behavior
- plugin integration
- service availability
- model wrapping/modification behavior
- WSL-specific runtime fragility

That is exactly the kind of work that can burn many hours while producing only modest portfolio value.

#### 2. Gazebo has limited scientific credibility

Gazebo is stronger than synthetic benchmarks, but only modestly.

It still shares important weaknesses:

- it is simulated
- the dynamics and noise model are still engineered by the project author
- it is not a recognized external benchmark in the way public datasets are

So even a working Gazebo score does not move Aegis nearly as far as a public dataset would.

#### 3. Portfolio and interview value are mixed

Gazebo helps for:

- ROS2 familiarity
- simulation integration
- autonomy-stack plumbing

But the marginal value drops quickly once you already have:

- a working ROS2 localization stack
- a reproducible synthetic benchmark
- tests
- evaluation scripts

At that point, another week of Gazebo debugging is less impressive than a clean external validation backend.

#### 4. Graduate-school and research value are limited

For MS admissions and robotics labs, the strongest signal is not “I got Gazebo to run.”

The stronger signal is:

- “I designed a clean evaluation framework”
- “I validated on a recognized external benchmark”
- “I used the framework to study a meaningful localization question”

Gazebo is helpful, but it is not the most convincing evidence source.

### Conclusion on Phase 5

**Finishing Gazebo is still useful, but it is not clearly the highest-value engineering investment from here.**

If Gazebo can be finished quickly, it is worth doing.

If it continues to demand infrastructure debugging, it should be demoted behind public dataset support and evaluation framework generalization.

## Part 3: Compare Future Evaluation Backends

### Evaluation criteria

For each backend:

- engineering effort
- scientific credibility
- hiring value
- graduate-school value
- reproducibility
- long-term maintainability
- fit with Aegis

### 1. Synthetic benchmark

Engineering effort:
- Already done and strong.

Scientific credibility:
- Low to moderate.
- Good for controlled ablations, weak as external evidence.

Hiring value:
- Good for robotics software interviews because it proves ownership and reproducibility.

Graduate-school value:
- Useful as a baseline, not sufficient as the main evidence source.

Reproducibility:
- Excellent.

Long-term maintainability:
- Excellent.

Fit with Aegis:
- Excellent as the foundational baseline.

Verdict:
- Must remain the baseline, but should not remain the strongest validation layer.

### 2. Gazebo

Engineering effort:
- Moderate to high, currently trending high because of infrastructure fragility.

Scientific credibility:
- Moderate.

Hiring value:
- Good for ROS2 and simulation integration.

Graduate-school value:
- Moderate, but weaker than external datasets.

Reproducibility:
- Mixed.
- Better than hardware, worse than synthetic, often sensitive to environment.

Long-term maintainability:
- Moderate to weak.
- Simulator/tooling drift is a real burden.

Fit with Aegis:
- Good as an integration layer, not ideal as the flagship evaluation layer.

Verdict:
- Useful secondary backend, but not the strongest next flagship backend.

### 3. EuRoC MAV

Engineering effort:
- Moderate.
- Requires dataset adapter work, timestamp handling, frame conventions, and likely partial measurement-model adaptation.

Scientific credibility:
- High.
- Recognized robotics benchmark with real sensor data.

Hiring value:
- High.
- Signals real robotics evaluation discipline.

Graduate-school value:
- High.

Reproducibility:
- High if documented well.

Long-term maintainability:
- Good.

Fit with Aegis:
- Strong if Aegis expands beyond planar wheel-odom assumptions toward generic trajectory evaluation and sensor adapters.

Verdict:
- One of the strongest future backends if the estimator/data assumptions are adapted carefully.

### 4. KITTI

Engineering effort:
- Moderate to high.
- Best fit if you support vehicle-style odometry/localization assumptions.

Scientific credibility:
- High.

Hiring value:
- High, especially for autonomy and AV-adjacent roles.

Graduate-school value:
- High.

Reproducibility:
- High.

Long-term maintainability:
- Good.

Fit with Aegis:
- Good, but less natural than EuRoC if the repo remains focused on general robotics localization rather than car-like autonomy.

Verdict:
- Strong option, especially if the repo wants an autonomy-vehicle flavor.

### 5. TUM RGB-D

Engineering effort:
- Moderate, but less natural for the current project.

Scientific credibility:
- Good in perception/SLAM contexts.

Hiring value:
- Good for perception-heavy roles.

Graduate-school value:
- Good.

Reproducibility:
- High.

Long-term maintainability:
- Good.

Fit with Aegis:
- Weak to moderate for the current repo.
- TUM RGB-D naturally belongs more to vision/SLAM evaluation than to wheel-odom/IMU localization benchmarking.

Verdict:
- Not the best first public dataset for Aegis unless the project pivots strongly toward visual correction.

### 6. Isaac Sim

Engineering effort:
- High.

Scientific credibility:
- Moderate.

Hiring value:
- Good in some robotics stacks, especially NVIDIA-heavy ecosystems.

Graduate-school value:
- Moderate.

Reproducibility:
- Mixed.

Long-term maintainability:
- Moderate to weak.

Fit with Aegis:
- Weaker than public datasets.

Verdict:
- Lower priority than external datasets and probably lower priority than a stable Gazebo path.

### 7. Recorded ROS bags

Engineering effort:
- Low to moderate.

Scientific credibility:
- Variable.
- Depends entirely on data quality and documentation.

Hiring value:
- Good if the bags are real robot or realistic field recordings.

Graduate-school value:
- Moderate to high if the data is strong and well documented.

Reproducibility:
- Good if bags are bundled or easily downloadable.

Long-term maintainability:
- Good.

Fit with Aegis:
- Excellent.
- This is one of the most natural extensions because it preserves the ROS2 pipeline directly.

Verdict:
- Very strong practical backend, especially if a public ROS bag source can be identified.

### 8. Other publicly available localization datasets

Best-fit examples would be datasets with:

- IMU
- wheel odometry or motion priors
- optional pose ground truth
- ROS-amenable formatting or convertible logs

Engineering effort:
- Moderate.

Scientific credibility:
- High if the dataset is known and well cited.

Hiring value:
- High.

Graduate-school value:
- High.

Reproducibility:
- High if ingestion is documented cleanly.

Long-term maintainability:
- Good.

Fit with Aegis:
- Usually stronger than simulator-specific backends.

Verdict:
- Public real-data evaluation is the best way to make Aegis feel serious.

### Recommended backend order

1. Synthetic benchmark
2. Recorded ROS bags or a ROS-friendly public real-data backend
3. EuRoC MAV
4. Gazebo
5. KITTI
6. TUM RGB-D
7. Isaac Sim

### Why this order

- Synthetic is already the stable baseline.
- Recorded ROS bags are the most natural bridge from current ROS2 infrastructure to real-data credibility.
- EuRoC is a high-value public benchmark with stronger research credibility than Gazebo.
- Gazebo remains useful, but mainly as an integration layer, not the best flagship evidence layer.

## Part 4: Long-Term Architecture

The right long-term architecture is:

**one evaluation core, many backend adapters**

### Proposed directory structure

```text
ros2_ws/
  src/
    aegis_core/
    aegis_ros/
    aegis_msgs/

scripts/
  evaluate_trajectory.py
  run_fake_benchmark_campaign.py
  run_gazebo_validation.py
  run_backend_eval.py

benchmarks/
  synthetic/
    scenarios/
    configs/
  gazebo/
    configs/
  datasets/
    euroc/
    kitti/
    rosbags/

adapters/
  synthetic/
  gazebo/
  euroc/
  kitti/
  rosbag/

results/
  synthetic/
  gazebo/
  datasets/
    euroc/
    kitti/
    rosbag/
```

### Core interface idea

Each backend should produce the same normalized artifacts:

- `ground_truth.csv`
- `ekf.csv`
- `ukf.csv`
- `pf.csv`
- backend metadata
- scenario metadata

That means the evaluation framework should not care whether the source was:

- fake sensor publisher
- Gazebo
- EuRoC adapter
- rosbag replay
- future hardware log

### Adapter responsibilities

Each backend adapter should be responsible for:

- data ingestion
- topic or file mapping
- frame normalization
- timestamp normalization
- routing data into the shared estimator/evaluation path

Examples:

`synthetic adapter`

- launches fake publisher
- writes canonical CSV outputs

`Gazebo adapter`

- launches simulation
- publishes or captures simulator truth
- writes canonical CSV outputs

`dataset adapter`

- converts source dataset into canonical topic or CSV form
- handles frame transforms and timestamp association

`hardware adapter`

- replays recorded robot data or captures live logs into the same canonical format

### Evaluation pipeline

The shared pipeline should be:

1. backend run
2. canonical trajectory export
3. per-estimator metric computation
4. optional plots
5. summary generation
6. metadata capture

### Metrics

All backends should report the same primary metrics where possible:

- ATE RMSE
- final drift
- yaw RMSE
- update rate
- number of valid samples

Optional backend-specific metrics:

- failure rate under dropout
- robustness under degraded sensor modes
- runtime cost
- relocalization recovery quality

### Metadata

Every run should capture:

- backend type
- scenario or dataset name
- duration
- estimator configuration
- seed where relevant
- commit hash
- run timestamp
- source command
- frame assumptions
- whether truth is simulated, dataset-based, or hardware-based

### Benchmark organization

The benchmark organization should separate:

- backend
- scenario or dataset
- estimator outputs
- metrics
- metadata

A good target shape is:

```text
results/
  synthetic/
    low_noise/
    high_noise_dropout/
  gazebo/
    burger_circle/
  datasets/
    euroc/
      MH_01/
      V1_01/
```

This is cleaner than mixing all backends into one flat metrics directory.

## Part 5: Research Direction

### Recommendation

The highest-value major extension is:

**visual or landmark-assisted correction for drift recovery**

### Why this is the strongest extension

Right now Aegis proves:

- classical filters are implemented
- synthetic evaluation is reproducible
- the framework can compare estimators

What it does not yet prove is a compelling research question.

Visual or landmark-assisted correction adds one immediately:

**How much can intermittent exteroceptive correction improve localization robustness under drift, dropout, and noise?**

That question is:

- technically meaningful
- easy to explain in interviews
- natural for robotics research
- compatible with public datasets
- compatible with future hardware

### Why this is stronger than the alternatives

`uncertainty estimation`

- valuable, but harder to make visibly compelling without stronger external validation first

`adaptive covariance tuning`

- interesting, but feels more incremental and less legible to recruiters and faculty

`more IMU fusion work`

- useful, but the repo already has IMU fusion

`dropout robustness alone`

- already partly present in the synthetic pipeline

### Best concrete framing

The most valuable research version of Aegis is:

- classical dead-reckoning and filter baseline
- shared evaluation framework
- intermittent visual or landmark correction module
- experiments on drift, dropout, and recovery

That turns the project from “I implemented filters” into “I built a platform and used it to study a localization question.”

## Part 6: Decision

### Recommendation

**B. Pivot to public dataset support first**

### Why this is the best choice

#### 1. It gives the biggest credibility jump

Public real-data evaluation moves Aegis from:

- well-engineered simulation benchmark

to:

- externally validated localization platform

That jump matters much more for internships, admissions, and labs than a fully debugged Gazebo integration.

#### 2. It has better return on engineering time

Gazebo Phase 5 is now clearly in infrastructure-risk territory.

The remaining effort is likely to involve:

- simulator spawn quirks
- model/plugin integration debugging
- environment-specific behavior

Those hours are unlikely to generate as much visible project value as implementing a clean dataset adapter.

#### 3. It fits the long-term identity better

If Aegis is a benchmarking and research platform, the strongest next move is not “more simulator glue.”

It is:

- backend modularity
- one real-data backend
- then a research extension

#### 4. Gazebo should be reframed, not discarded

This does **not** mean Gazebo work was wasted.

Instead:

- keep Gazebo as an integration backend
- keep it explicitly secondary
- return to it later if it becomes easy to stabilize

### Practical next sequence

1. Keep the current Gazebo work in a technically honest in-progress state.
2. Design the backend adapter structure explicitly.
3. Implement one public real-data backend first.
4. Then add the visual or landmark-assisted correction extension.
5. Revisit Gazebo only if it becomes easy to stabilize or useful for demos.

### Final decision statement

The highest-value direction is:

**pivot from “finish Gazebo at all costs” to “add public dataset support first, while preserving Gazebo as a secondary integration backend.”**

That is the best trade for:

- engineering time
- research credibility
- hiring value
- graduate-school value
- long-term project coherence
