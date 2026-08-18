# Canonical Run Schema

This document defines the minimal canonical run layout for Aegis benchmark backends.

The purpose is to let synthetic runs, future EuRoC runs, and later backends produce the same evaluation artifacts without forcing a full architecture rewrite first.

## Design Principles

- keep the schema small
- preserve the current estimator implementations
- preserve the current synthetic benchmark workflow
- make one public dataset backend possible without redoing the evaluation layer again

## Canonical Directory Layout

Each backend run should target:

```text
results/
  <backend>/
    <benchmark_name>/
      <run_name>/
        manifest.json
        metadata.json
        normalized/
          ground_truth.csv
          ekf.csv
          ukf.csv
          pf.csv
        metrics/
        plots/
        logs/
```

## Required Normalized Trajectory Files

The canonical normalized outputs are:

- `ground_truth.csv`
- `ekf.csv`
- `ukf.csv`
- `pf.csv`

Each CSV must use:

```text
timestamp,x,y,yaw
```

Current evaluation and plotting code assumes exactly these columns.

## Required Metadata Files

### `manifest.json`

Describes the intended run:

- backend name
- benchmark or sequence name
- estimator set
- config assumptions
- command or launch arguments

### `metadata.json`

Describes what actually happened:

- timestamp
- seed when relevant
- commit hash when available
- runtime notes
- reduction assumptions
- truth source type

## Phase A Scope Note

Phase A does not require all existing scripts to emit this exact structure yet.

Phase A requires:

- the schema to be defined
- helper code to create it
- the evaluation package to be reusable against it

Phase B should use this schema directly for the first EuRoC backend.
