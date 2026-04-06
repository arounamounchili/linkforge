# LinkForge Core

Universal Robotics Interoperability Library.
This package contains the core logic for LinkForge, independent of any platform (Blender, etc.).

It provides robust, dependency-minimized parsing and generation of robot descriptions (URDF, Xacro, SRDF), physics engine utilities, and validation models.

## Structure

- `src/linkforge_core/`: The main source code.
  - `models/`: Data structures representing robots, joints, links, and semantic info (SRDF).
  - `parsers/`: Parsers for URDF, XACRO, and SRDF formats.
  - `generators/`: Exporters back to XML formats (URDF, Xacro, SRDF).
  - `composer/`: Robot assembly logic and factory patterns.
  - `physics/`: Math and kinematics utilities for inertia and joint dynamics.
  - `validation/`: Robust schema validation for robot data.
  - `utils/`: Common helpers and mathematical operations.
  - `base.py`, `exceptions.py`, `logging_config.py`: Core interfaces, error handling, and logging.

## Development

The core library is built and managed using [`uv`](https://docs.astral.sh/uv/).

```bash
# Install dependencies
uv sync

# Run core tests (execute from the project root directory)
uv run pytest tests/unit/core
```
