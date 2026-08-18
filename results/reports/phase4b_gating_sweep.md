# Phase 4b Gating Sweep

## Diagnosis

- The Phase 4 catastrophic run pattern is consistent with startup poisoning: a corrupted initialization can make almost every later pose correction look inconsistent and trigger a rejection spiral.
- To test that directly, this sweep compares gating with corruption active from the first update versus the same gate with corruption delayed until after initialization.

| Diagnosis Metric | Startup Poison Gate95 | Delayed Corruption Gate95 |
|---|---:|---:|
| EKF ATE RMSE | 0.0656 | 0.0663 |
| UKF ATE RMSE | 0.0656 | 0.0663 |
| EKF rejection rate | 0.0933 | 0.0717 |
| UKF rejection rate | 0.0936 | 0.0718 |

## Delayed-Corruption Sweep

| Scenario | EKF ATE | UKF ATE | EKF reject | UKF reject | EKF TPR | EKF FPR |
|---|---:|---:|---:|---:|---:|---:|
| baseline_p05_mild | 0.0709 | 0.0709 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| baseline_p05_strong | 0.0833 | 0.0833 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| baseline_p10_mild | 0.0873 | 0.0872 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| baseline_p10_strong | 0.1147 | 0.1147 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| gate_95_p05_mild | 0.0664 | 0.0664 | 0.0317 | 0.0317 | 0.7943 | 0.0000 |
| gate_95_p05_strong | 0.0636 | 0.0636 | 0.0425 | 0.0426 | 0.9296 | 0.0000 |
| gate_95_p10_mild | 0.0664 | 0.0664 | 0.0750 | 0.0752 | 0.8463 | 0.0000 |
| gate_95_p10_strong | 0.0634 | 0.0634 | 0.0883 | 0.0886 | 0.9719 | 0.0000 |
| gate_999_p05_mild | 0.0657 | 0.0657 | 0.0308 | 0.0309 | 0.6548 | 0.0000 |
| gate_999_p05_strong | 0.0637 | 0.0637 | 0.0425 | 0.0426 | 0.8540 | 0.0000 |
| gate_999_p10_mild | 0.0635 | 0.0634 | 0.0708 | 0.0710 | 0.7213 | 0.0000 |
| gate_999_p10_strong | 0.0650 | 0.0650 | 0.0833 | 0.0835 | 0.8610 | 0.0000 |
| gate_99_p05_mild | 0.0654 | 0.0654 | 0.0467 | 0.0451 | 0.8574 | 0.0000 |
| gate_99_p05_strong | 0.0642 | 0.0641 | 0.0367 | 0.0367 | 0.8774 | 0.0000 |
| gate_99_p10_mild | 0.0689 | 0.0690 | 0.0792 | 0.0794 | 0.7787 | 0.0000 |
| gate_99_p10_strong | 0.0649 | 0.0649 | 0.0767 | 0.0768 | 0.8877 | 0.0000 |

## Interpretation

- If delayed-corruption gating performs much better than startup-poison gating, the original Phase 4 failure mode was at least partly an initialization problem rather than ordinary update rejection alone.
- The threshold sweep then shows whether a fixed chi-square gate can achieve a useful tradeoff between true outlier rejection and false rejection of valid corrections.
- A good operating region would show lower ATE than the matching no-gate baseline, high true-positive rejection, and meaningfully lower false-positive rejection than the failed Phase 4 configuration.

## Artifacts

- Machine-readable summary: `results/phase4b_gating_sweep/summary.json`
- Run-level artifacts: `results/phase4b_gating_sweep/<scenario>/repeats/run_###/`

