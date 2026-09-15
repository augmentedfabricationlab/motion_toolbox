# Repository workflow

- Preserve collision coverage and exact planner results when optimizing; benchmark and test changes.
- Increment the package patch version for each implementation-change commit. Keep pyproject.toml, package __version__, and ROS package.xml (when present) consistent.
- Update CHANGELOG.md in the same commit as every package version bump. Add a dated entry matching the new version, summarizing user-visible changes, fixes, compatibility/dependency changes, and relevant validation. Keep newest versions first; move applicable Unreleased notes into the version entry. Documentation-only work may stay under Unreleased without a version bump. Do not describe uncommitted work as released or claim validation that was not performed.
- Commit tested work in small logical checkpoints. Do not leave a whole session uncommitted.
- Publish authorized releases to augmentedfabricationlab. Never commit local research_runs, environment files, credentials, or generated robot/input geometry.
- Include the package version and loaded-code fingerprints in recording metadata when changing recording behavior.
