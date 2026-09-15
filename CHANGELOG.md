# Changelog

Notable changes by package version, newest first. Historical entries were
reconstructed from Git commits and `pyproject.toml`; dates are commit dates,
not evidence of a published release. Commit references identify the source.

## Unreleased

### Documentation

- Add this version history and require changelog updates with future version bumps.

## 0.1.6 - 2026-09-15

Source commit: `6350b45`.

- Simplify sparse mobile paths in world XY, with a default tolerance of 0.05 m,
  so small ripples do not force extra base-motion keyframes.
- Retain projected wall-normal heading changes; optionally retain height changes.
  Validate every original 3D target after interpolation.
- Use projected XY progress for interpolation. Index and distance gap limits now
  default to disabled; `sampling: "legacy"` restores the earlier selector.
- Expose `xy_tolerance` in the dedicated Grasshopper component in model units.

## 0.1.5 - 2026-09-15

Source commit: `6569a0c`.

- Generate mobile candidates using stationary placement constraints: the arm-base
  origin must be behind projected target +Z and within 1.75 m in XY, accounting
  for the mounting offset. Apply the same constraints to interpolated targets.
- Order region candidates by standoff and check base-body collisions before IK.
  Sparse search defaults to four feasible bases per searched target; this cap
  can miss connected paths. Dense mode searches the complete sampled region.
- Reuse candidate checks during fallback and fix planning/recording diagnostics.
- Replace dedicated-component offset/seed-distance search controls with region
  sampling controls (`grid_spacing`, `yaw_steps`, and `mobile_options`).

## 0.1.4 - 2026-09-15

Source commit: `fa3a823`.

- Add `examples/grasshopper_mobile_base.py`, a dedicated mobile base-motion
  component with automatic footprint seeds and optional supplied seed paths.
- Return a base plane, joint plan, and named configuration for every original
  target on success; expose sparse search and start-base settings.

## 0.1.3 - 2026-09-15

Source commit: `55787fd`.

- Add sparse mobile base planning and Grasshopper `mobile_options` integration.
- Search retained keyframes, interpolate base motion, and solve arm IK and check
  constraints at every original target; fall back to dense search on failure.
- Add sparse-planning tests and a mobile planning benchmark.

## 0.1.2 - 2026-09-15

Source commit: `edc5d0d`.

- Add collision-cache grouping for equivalent bounded joint turns; advance the
  collision API version to 10 and refine graph/planning implementation.
- Record loaded module versions, file hashes, and loaded-code fingerprints to
  distinguish cached Rhino imports from files on disk.
- Advance recording version to 3, save recording errors, and improve repository
  metadata collection. Add recording tests and revision benchmark results.

## 0.1.1 - 2026-09-15

Source commit: `1040b33`.

- Add projected stationary placement regions, bounded candidate validation,
  detailed failure diagnostics, and dedicated Grasshopper planning examples.
- Improve collision filtering and reuse checks for full-turn-equivalent joint
  configurations; optimize graph search while retaining bounded configurations.
- Add automatic research recording with SQLite runs, compressed artifacts,
  metrics export through `toolbox-metrics`, and a recording guide.
- Expand adapter, collision, planner, Grasshopper, and recording regression
  coverage; add validation and benchmark artifacts.
- Establish package versioning and checkpoint-commit workflow instructions.

## 0.1.0 - 2026-09-10

Source commit: `8b3d6fa`.

- Extract the reusable motion planning package from the existing project code.
- Provide numeric IK, optional PyBullet collision checking, layered motion
  planning, stationary/mobile base planning, rolling replanning, and the
  `motion-plan` command, with optional COMPAS integration.
