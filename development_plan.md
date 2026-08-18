# Aegis Localization Development Plan

## North Star

Aegis Localization should become a modular localization benchmarking and evaluation framework that starts with a reproducible synthetic baseline, extends to honest simulator evaluation, and later supports public datasets and vision- or landmark-assisted localization.

Synthetic benchmarking is not the final goal. It is the first credibility gate.

## What The Project Should Prove

- EKF, UKF, and particle-filter localization are implemented and understood in-house.
- The evaluation pipeline is trustworthy and reproducible.
- Synthetic, simulator, and future dataset backends can share one metrics framework.
- Repository claims are backed by working commands and defensible evidence.
- The project can grow into a research platform rather than staying a synthetic-only demo.

## Phase 1: Establish Official Repo Truth

Goal:
Decide what the project officially is before changing behavior.

Work:

- Promote or discard dirty local Gazebo and campaign work intentionally.
- Separate official evidence from stale or broken artifacts.
- Define supported evaluation backends:
  - synthetic
  - Gazebo
  - future dataset adapters
- Define what claims are currently allowed.

Exit criteria:

- No ambiguity about which files are official.
- No docs implying unsupported functionality.
- Clear repo identity.

## Phase 2: Repair The Baseline

Goal:
Make one end-to-end synthetic benchmark path actually work.

Work:

- Fix stale WSL and absolute path assumptions.
- Fix current ground-truth and output handling.
- Make `scripts/evaluate_trajectory.py` robust on current outputs.
- Make README commands run from the current repo layout.
- Verify EKF, UKF, and PF all evaluate through the same path.

Exit criteria:

- Documented synthetic benchmark commands work.
- Metrics generate cleanly for all three estimators.
- No manual cleanup is required between runs.

## Phase 3: Harden The Evaluation Framework

Goal:
Make the pipeline trustworthy, repeatable, and extensible.

Work:

- Add deterministic seeds where appropriate.
- Capture run metadata:
  - scenario
  - duration
  - seed
  - estimator
  - timestamp
  - commit hash when practical
- Improve result schema and output organization.
- Ensure future backends can reuse the same metric pipeline.

Exit criteria:

- Reruns are comparable.
- Results are traceable.
- Evaluation structure is backend-agnostic.

## Phase 4: Strengthen Regression Protection

Goal:
Stop future changes from silently breaking estimators or evaluation.

Work:

- Add estimator regression tests for:
  - zero motion
  - angle wrap
  - dropout behavior
  - covariance and PSD sanity
  - repeated updates
  - initialization edge cases
- Add evaluation tests for:
  - timestamp alignment
  - empty and malformed CSV handling
  - metric sanity

Exit criteria:

- Core estimator and evaluation edge cases are covered.
- Failures point clearly to the broken layer.

## Phase 5: Resolve Gazebo Honestly

Goal:
Make Gazebo either real evidence or clearly demo-only.

Acceptable outcomes:

- Gazebo scoring works with usable ground-truth-backed metrics.
- Gazebo is explicitly described as integration or demo validation only.

Work:

- Verify ground-truth bridge behavior.
- Verify logger output under Gazebo.
- Verify whether metrics are meaningful.
- Update docs to match the truth.

Exit criteria:

- No ambiguous simulator claims.
- Gazebo status is technically defensible.

## Phase 6: Add Public Dataset Support

Goal:
Move from synthetic and simulated evaluation to stronger research evidence.

Work:

- Make a dataset adapter interface.
- Pick a first public dataset later, likely a localization-relevant one.
- Reuse the same metrics pipeline.
- Document exact commands and limitations.

Exit criteria:

- At least one non-synthetic backend can run through the shared evaluation framework.

## Phase 7: Add The Research Extension

Goal:
Make the repo intellectually stronger, not just better organized.

Best extension:

- vision- or landmark-assisted correction into localization

Possible directions:

- intermittent visual pose correction
- landmark observation updates
- exteroceptive correction under drift, noise, and dropout

Exit criteria:

- The project evolves from a filter comparison into a more compelling localization research platform.

## Priority Order

1. Phase 1: official repo truth
2. Phase 2: synthetic baseline repair
3. Phase 3: evaluation hardening
4. Phase 4: regression protection
5. Phase 5: Gazebo resolution
6. Phase 6: public dataset support
7. Phase 7: research extension

## What Not To Do Yet

- Do not overbuild synthetic-only infrastructure.
- Do not jump to large public datasets before the current metrics pipeline is trustworthy.
- Do not present Gazebo as stronger evidence than it is.
- Do not add a research extension before the baseline is reproducible.

## Immediate Next Sprint

- Finalize official inclusion of Gazebo and campaign files.
- Repair stale path assumptions.
- Repair ground-truth and evaluation flow.
- Get one clean documented synthetic benchmark reproduction working.
- Verify EKF, UKF, and PF all produce valid metrics through the same path.
