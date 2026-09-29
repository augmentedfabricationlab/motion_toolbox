# Grasshopper arm/base path export

Load `examples/grasshopper_export_paths.py` in a Rhino 8 **Python 3** component.
Use these exact parameter names. This is the component entry point; the helper
in `src/motion_toolbox/utilities/path_export.py` contains reusable functions.

Connect a **Panel to the built-in `out` output** to see the export location.
The component prints its summary, destination, both file paths and any warnings.
On failure it prints an error; in preview mode it states that no files were written.

## Required inputs

| Name | Access | Type hint | Connect / meaning |
| --- | --- | --- | --- |
| `tcp_planes` | List | Plane | Connect the planner's `planned_tcp`: the TCP planes selected by the planner, in order. |
| `base_planes` | List | Plane | Corresponding planned base planes, in the same order and with the same count. |
| `speed` | Item | float / Number | Positive TCP translation speed in **cm/s**. For example, `4` means 4 cm/s. |

Plane coordinates are assumed to be **metres** unless `model_units_to_metres`
is supplied. Use List access for both plane inputs so the component exports the
whole path together, rather than being called separately for each plane.

## Optional inputs

These may be removed, marked Optional, or left unconnected to use the defaults.
All use **Item** access.

| Name | Type hint | Default | Meaning |
| --- | --- | --- | --- |
| `write_files` | bool / Boolean | `True` | Export files. `False` computes timing and diagnostics without writing path files. |
| `model_units_to_metres` | float / Number | `1.0` | Scale plane positions to metres; use `0.001` for millimetres. Speed remains in cm/s. |
| `frame_id` | str / Text | `vicon_world` | Coordinate-frame name written to both JSON files. |
| `sanity_checks` | bool / Boolean | `True` | Retain the original checks: at least one TCP above 1 m, and base XY bounding-box diagonal at least 0.9 m. Disable for intentionally smaller paths. |
| `documents_folder` | str / Text | Windows Documents | Parent destination folder. The component creates a new timestamped subfolder inside it. |
| `toolbox_src` | str / Text | Adjacent `src`, otherwise installed toolbox | Optional path to the repository's `src` directory, not a Python file. Usually omit. |

Every recompute with `write_files=True` creates a new export folder. Earlier
exports are preserved using numbered suffixes when the minute-based name exists.

## Outputs

Keep the built-in `out` output. Add or rename other outputs below as needed.
Outputs do not require input type hints.

| Name | Value | Meaning |
| --- | --- | --- |
| `out` | Printed text | Summary, destination folder, arm/base file paths, warnings and errors. Connect a Panel. |
| `tcp_file` | Text | Full path to `arm_path.json`; empty in preview or on error. |
| `base_file` | Text | Full path to `base_path.json`; empty in preview or on error. |
| `export_folder` | Text | Full path to the newly created export folder; empty in preview or on error. |
| `time_seconds` | Number list | Relative timestamp for each paired pose, in seconds. Starts at zero. |
| `duration_seconds` | Number | Total duration in seconds. |
| `diagnostics` | Text list | Geometry/unit warnings, repeated-timestamp notice, or export error. |
| `status` | Text | One-line success, preview or error summary. |
| `result` | Object | Python dictionary containing the JSON payloads, timestamps, diagnostics and file paths. Use the named text outputs for readable Panels. |
| `version` | Text | Loaded motion-toolbox package version. |

Example `out` text:

```text
Exported 1760 paired poses; 2524.411 s; arm/base timestamps identical. Saved to: C:\Users\david\Documents\260929_1300_robot_path
Arm path: C:\Users\david\Documents\260929_1300_robot_path\arm_path.json
Base path: C:\Users\david\Documents\260929_1300_robot_path\base_path.json
```

## Timing and export format

Time increments are the **3D distance between consecutive TCP origins** divided
by `speed / 100` metres per second. Both files use this same timestamp list at
every index. Base planes are not moved, resampled or rotated by the exporter.
Different base and TCP travel distances therefore produce different speeds even
though their timestamps match.

Files contain metre positions, normalized `x,y,z,w` quaternions, `frame_id`, and
relative timestamps as integer `sec` / `nanosec`. Repeated TCP origins receive no
extra duration, including rotation-only changes; the component reports this in
`diagnostics` and `out`. Exporting does not check IK, collisions or motion limits.
