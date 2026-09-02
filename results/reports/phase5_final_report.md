# Phase 5: Delayed And Intermittent Correction

## Scope

- Phase 5B compares the same delayed correction stream under naive arrival-time fusion and timestamp-aware replay.
- ATE describes online published-trajectory error. Final drift describes the final current-state error after all received corrections have been incorporated.
- Phase 5C holds the replay policy fixed and varies correction availability and quality with repeated seeds.
- Trajectory metrics use the completed 3-repeat matrices. Pose-NIS logging was subsequently verified in a focused run; it does not alter estimator state or trajectory metrics.
- Combined-degraded pose-NIS aggregation is intentionally shown as unavailable for the pre-instrumentation run, not imputed from estimator counters.

## Phase 5B: Naive Fusion Versus Replay

| Latency | Estimator | Naive online ATE | Replay online ATE | Naive final drift | Replay final drift | Drift improvement |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| latency_0000ms | EKF | 0.0577 | 0.0577 | 0.0448 | 0.0448 | -0.0000 |
| latency_0000ms | UKF | 0.0576 | 0.0577 | 0.0448 | 0.0448 | 0.0000 |
| latency_0000ms | PF | 0.0481 | 0.0482 | 0.0296 | 0.0293 | 0.0003 |
| latency_0100ms | EKF | 0.0647 | 0.0592 | 0.0593 | 0.0500 | 0.0093 |
| latency_0100ms | UKF | 0.0647 | 0.0592 | 0.0593 | 0.0500 | 0.0093 |
| latency_0100ms | PF | 0.0497 | 0.0463 | 0.0415 | 0.0407 | 0.0008 |
| latency_0500ms | EKF | 0.2290 | 0.0923 | 0.1735 | 0.0526 | 0.1208 |
| latency_0500ms | UKF | 0.2290 | 0.0923 | 0.1735 | 0.0526 | 0.1208 |
| latency_0500ms | PF | 0.2247 | 0.0556 | 0.1357 | 0.0444 | 0.0913 |
| latency_1000ms | EKF | 0.4402 | 0.0968 | 0.3005 | 0.0552 | 0.2453 |
| latency_1000ms | UKF | 0.4404 | 0.0968 | 0.3005 | 0.0552 | 0.2453 |
| latency_1000ms | PF | 0.4450 | 0.0507 | 0.3481 | 0.0360 | 0.3122 |

## Phase 5C: Replay Degradation Campaign

| Scenario | Estimator | ATE mean +/- std | Final drift mean +/- std | Yaw RMSE mean +/- std |
| --- | --- | ---: | ---: | ---: |
| reference | EKF | 0.0539 +/- 0.0098 | 0.0559 +/- 0.0103 | 0.0248 +/- 0.0025 |
| reference | UKF | 0.0539 +/- 0.0098 | 0.0559 +/- 0.0103 | 0.0248 +/- 0.0025 |
| reference | PF | 0.0483 +/- 0.0023 | 0.0261 +/- 0.0062 | 0.0305 +/- 0.0015 |
| low_frequency | EKF | 0.0829 +/- 0.0111 | 0.0768 +/- 0.0434 | 0.0337 +/- 0.0098 |
| low_frequency | UKF | 0.0848 +/- 0.0104 | 0.0767 +/- 0.0435 | 0.0306 +/- 0.0128 |
| low_frequency | PF | 0.0564 +/- 0.0078 | 0.0791 +/- 0.0400 | 0.0369 +/- 0.0091 |
| random_dropout | EKF | 0.0754 +/- 0.0033 | 0.0778 +/- 0.0343 | 0.0323 +/- 0.0062 |
| random_dropout | UKF | 0.0754 +/- 0.0033 | 0.0778 +/- 0.0343 | 0.0323 +/- 0.0062 |
| random_dropout | PF | 0.0488 +/- 0.0020 | 0.0297 +/- 0.0170 | 0.0360 +/- 0.0049 |
| delayed | EKF | 0.0910 +/- 0.0034 | 0.0453 +/- 0.0202 | 0.0291 +/- 0.0054 |
| delayed | UKF | 0.0910 +/- 0.0034 | 0.0453 +/- 0.0202 | 0.0291 +/- 0.0054 |
| delayed | PF | 0.0514 +/- 0.0019 | 0.0223 +/- 0.0088 | 0.0311 +/- 0.0047 |
| noisy | EKF | 0.0845 +/- 0.0159 | 0.0648 +/- 0.0330 | 0.0286 +/- 0.0049 |
| noisy | UKF | 0.0845 +/- 0.0159 | 0.0648 +/- 0.0330 | 0.0286 +/- 0.0049 |
| noisy | PF | 0.0919 +/- 0.0034 | 0.0612 +/- 0.0185 | 0.0371 +/- 0.0048 |
| corrupted_no_gate | EKF | 0.2342 +/- 0.0414 | 0.0619 +/- 0.0331 | 0.0800 +/- 0.0241 |
| corrupted_no_gate | UKF | 0.2334 +/- 0.0421 | 0.0619 +/- 0.0331 | 0.0805 +/- 0.0246 |
| corrupted_no_gate | PF | 0.0699 +/- 0.0096 | 0.0382 +/- 0.0073 | 0.0364 +/- 0.0061 |
| corrupted_gated | EKF | 0.0709 +/- 0.0059 | 0.0798 +/- 0.0206 | 0.0243 +/- 0.0022 |
| corrupted_gated | UKF | 0.0708 +/- 0.0058 | 0.0798 +/- 0.0206 | 0.0241 +/- 0.0024 |
| corrupted_gated | PF | 0.0646 +/- 0.0038 | 0.0913 +/- 0.0602 | 0.0338 +/- 0.0018 |
| blackout_recovery | EKF | 0.0746 +/- 0.0076 | 0.0518 +/- 0.0185 | 0.0291 +/- 0.0042 |
| blackout_recovery | UKF | 0.0746 +/- 0.0077 | 0.0518 +/- 0.0185 | 0.0291 +/- 0.0042 |
| blackout_recovery | PF | 0.0404 +/- 0.0044 | 0.0372 +/- 0.0205 | 0.0332 +/- 0.0064 |
| combined_degraded | EKF | 0.0967 +/- 0.0178 | 0.1107 +/- 0.0258 | 0.0314 +/- 0.0107 |
| combined_degraded | UKF | 0.0967 +/- 0.0178 | 0.1107 +/- 0.0258 | 0.0314 +/- 0.0107 |
| combined_degraded | PF | 0.1076 +/- 0.0238 | 0.1148 +/- 0.0456 | 0.0428 +/- 0.0194 |

## Blackout Recovery

| Estimator | Error at blackout start | Peak during blackout | Error 1 s after recovery | Recovery from peak |
| --- | ---: | ---: | ---: | ---: |
| EKF | 0.0636 | 0.1384 | 0.0545 | 0.0839 |
| UKF | 0.0636 | 0.1384 | 0.0536 | 0.0848 |
| PF | 0.0313 | 0.0580 | 0.0392 | 0.0188 |

## Consistency And Gating

| Scenario | Estimator | Pose NIS in bounds | Pose NIS above upper bound | Pose-gating rejection rate |
| --- | --- | ---: | ---: | ---: |
| reference | EKF | 0.9344 | 0.0546 | 0.0000 |
| reference | UKF | 0.9344 | 0.0546 | 0.0000 |
| low_frequency | EKF | 1.0000 | 0.0000 | 0.0000 |
| low_frequency | UKF | 1.0000 | 0.0000 | 0.0000 |
| random_dropout | EKF | 0.9877 | 0.0000 | 0.0000 |
| random_dropout | UKF | 0.9877 | 0.0000 | 0.0000 |
| delayed | EKF | 1.0000 | 0.0000 | 0.0000 |
| delayed | UKF | 1.0000 | 0.0000 | 0.0000 |
| noisy | EKF | 0.9727 | 0.0109 | 0.0000 |
| noisy | UKF | 0.9727 | 0.0109 | 0.0000 |
| corrupted_no_gate | EKF | 0.6046 | 0.3954 | 0.0000 |
| corrupted_no_gate | UKF | 0.6047 | 0.3953 | 0.0000 |
| corrupted_gated | EKF | 0.8852 | 0.1038 | 0.1038 |
| corrupted_gated | UKF | 0.8847 | 0.1044 | 0.1044 |
| blackout_recovery | EKF | 0.9493 | 0.0362 | 0.0000 |
| blackout_recovery | UKF | 0.9493 | 0.0362 | 0.0000 |
| combined_degraded | EKF | n/a | n/a | n/a |
| combined_degraded | UKF | n/a | n/a | n/a |

## Interpretation Rules

- Do not call lower online ATE proof that replay removes latency: no estimator can improve a belief published before the delayed observation arrived.
- Treat lower replay final drift at the same latency as the direct evidence for timestamp-aware current-state reconstruction.
- Interpret PF separately: its resampling is stochastic, so exact state equivalence is not expected from the EKF/UKF deterministic correctness tests.
- Report negative or mixed gating results directly; the goal is a characterized robustness tradeoff, not a universal claim that gating helps.
