# Aegis Localization Development Plan

## North Star

Aegis is a robotics localization benchmarking and research platform: in-house EKF, UKF, and PF implementations evaluated through reproducible synthetic and recorded-data workflows, with a focused study of intermittent and delayed exteroceptive correction. It is not a hardware-validation claim, a full 6-DoF MAV localizer, or a Gazebo-first project.

## Current Status

- **Phase 1 - Recorded-data benchmark:** complete for a single EuRoC `MH_01_easy` planar proxy. The same canonical evaluation path now produces populated outputs, metadata, metrics, and an honest synthetic comparison.
- **Phase 2 - Repeated synthetic evidence:** complete. Campaigns vary deterministic seeds, preserve per-run artifacts, and aggregate mean and standard deviation for ATE, drift, yaw error, and update rate.
- **Phase 3 - Consistency analysis:** complete for EKF and UKF synthetic runs. NIS, chi-square bounds, and synthetic-only planar NEES summaries are exported.
- **Phase 4 - Robustness gating:** complete as a characterized result. Pose-update Mahalanobis gating is configurable and its false-rejection tradeoff is documented through a corruption/threshold sweep.
- **Phase 5 - Intermittent correction:** complete. Phase 5A establishes EKF/UKF replay correctness; Phase 5B quantifies naive late fusion versus timestamp-aware replay through 1 s latency; Phase 5C provides a 3-repeat correction-degradation campaign and blackout recovery evidence. Combined-degraded NIS is explicitly unavailable in the preserved pre-instrumentation matrix rather than inferred.
- **Phase 6 - External smoothing baseline:** not started. A GTSAM/iSAM2 baseline remains optional and should be added only after Phase 5 produces its primary result.

Gazebo remains integration/demo validation only because its ground-truth scoring bridge is not a reliable benchmark backend. Raw EuRoC files are intentionally local and ignored by Git; compact run summaries and reports are versioned instead.

## Evidence Map

- Recorded-data method and limitations: `docs/euroc_backend.md`
- Synthetic versus EuRoC comparison: `results/reports/phase2_low_noise_vs_euroc_mh01.md`
- Consistency analysis: `results/reports/phase3_estimator_consistency.md`
- Gating study: `results/reports/phase4_gating_report.md` and `results/reports/phase4b_gating_sweep.md`
- Intermittent-correction experiment: `results/reports/phase5_intermittent_correction.md`
- Delayed-replay correctness checks: `results/reports/phase5_correctness.md`
- Final replay and degradation evidence: `results/reports/phase5_final_report.md`

## Completed Phase 5 Evidence

### Phase 5B: Naive Late Fusion Versus Timestamp-Aware Replay

Goal: turn asynchronous replay support into a direct result.

- Apply the same delayed correction stream under two policies: fuse stale data on arrival, or rewind to the measurement timestamp and replay forward.
- Sweep a small set of latencies and report both online trajectory error and post-fusion/current-state error.
- Keep EKF and UKF as the primary comparison; report PF separately because its stochastic resampling means exact equivalence is not expected.

Result: replay reduces EKF/UKF mean terminal drift at 1 s latency from `0.3005 m` to `0.0552 m`, while online ATE remains distinct from post-fusion state error.

### Phase 5C: Compact Degradation Campaign

- Vary one factor at a time: correction frequency, dropout, latency, noise, and corruption/gating.
- Add one combined degraded condition and repeated seeds where randomness is present.
- Report ATE, final drift, yaw RMSE, NIS where meaningful, gating statistics, and recovery after dropout.

Result: outlier gating materially reduces EKF/UKF ATE in the injected-corruption condition, and all estimators have repeated-seed blackout and combined-degradation evidence.

### Phase 6: Optional External Baseline

- Add an offline GTSAM/iSAM2 smoother only as a comparator, using the canonical output format where practical.
- Start on synthetic data and extend to the EuRoC planar proxy only if the mapping remains methodologically honest.

Exit criteria: one comparison table of EKF, UKF, PF, and smoothing on a clearly bounded scenario.

## Scope Guardrails

- Do not add a second public dataset, a plugin framework, or a large reporting system before deciding whether the optional Phase 6 baseline is worth the scope.
- Do not describe Gazebo as quantitative validation or EuRoC as native 6-DoF MAV validation.
- Do not hide negative robustness or consistency results; they are part of the research evidence.
