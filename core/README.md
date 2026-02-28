# LinkForge Core

Universal Robotics Interoperability Library.
This package contains the core logic for LinkForge, independent of any platform (Blender, etc.).

It provides robust, dependency-minimized parsing and generation of robot descriptions (URDF, Xacro, MJCF), physics engine utilities, and validation models.

## Structure

- `src/linkforge_core/`: The main source code.
  - `models/`: Data structures representing robots, joints, links, and physics.
  - `parsers/`: Parsers for URDF, XACRO, and other robotic formats.
  - `generators/`: Exporters back to XML formats.
  - `physics/`: Math and kinematics utilities for inertia and joint dynamics.
  - `validation/`: Robust schema validation for robot data.
  - `utils/`: Common helpers and mathematical operations.

## Development

The core library is built and managed using [`uv`](https://docs.astral.sh/uv/).

```bash
# Install dependencies
uv sync

# Run core tests (execute from the project root directory)
uv run pytest tests/unit/core
```
