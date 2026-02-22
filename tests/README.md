# LinkForge Test Suite

LinkForge uses `pytest` for comprehensive testing of its core physics/kinematics engine and its Blender platform integration.

## Directory Structure

The test suite is divided into two main categories to ensure clear separation between pure logic and platform-specific behavior:

### 1. `unit/`
Isolated tests for individual components without external dependencies.
- **`unit/core/`**: Tests for robot models, URDF/Xacro parsers, and physics utilities. These run in standard Python.
- **`unit/blender/`**: Tests for Blender-specific utilities (scene helpers, gizmos, operators). These require a Blender environment.

### 2. `integration/`
End-to-end tests that verify the interaction between multiple components.
- **`integration/parsers/`**: Verifies complex URDF parsing and Xacro expansion scenarios.
- **`integration/blender/`**: Verifies the full Roundtrip process (Import → Scene Setup → Export).
- **`integration/features/`**: Validation of specific high-level features like Inertia calculations, Transmissions, and Sensor tags.

## How to Run Tests

### Standard Python Tests
To run core unit tests and parser integration tests:
```bash
pytest tests/unit/core tests/integration/parsers
```

### Blender-Dependent Tests
To run tests that require the Blender Python API (`bpy`), use the launcher from the project root:
```bash
python blender_launcher.py
```
*Note: Ensure your `BLENDER_PATH` environment variable is set or Blender is installed at its default location.*

## Best Practices for Contributors

1. **Use Fixtures**: Place shared test resources (example URDFs, mock robots) in `tests/conftest.py`. Prefer the `examples_dir` fixture over local path strings.
2. **Platform Isolation**: If a test doesn't explicitly need a 3D viewport or `bpy` data structures, place it in `core`.
3. **Roundtrip Integrity**: When adding support for a new URDF tag, always add a corresponding roundtrip test in `integration/blender/` to ensure export parity.
4. **Mocking**: Use `unittest.mock` to simulate Blender's asynchronous timers or IO operations where possible.
