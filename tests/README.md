# LinkForge Test Suite

LinkForge uses `pytest` to test both the core physics/kinematics engine and the Blender platform integration.

## Directory Structure

The test suite separates pure logic from platform-specific behavior:

### 1. `unit/`
Isolated tests for individual components avoiding external dependencies.
- **`unit/core/`**: Tests for robot models, parsers, and physics utilities. Runs in standard Python.
- **`unit/platforms/blender/`**: Tests for Blender utilities (scene helpers, visualization, operators). Requires a Blender runtime.

### 2. `integration/`
End-to-end tests verifying the interaction between multiple components.
- **`integration/core/`**: Verifies complex URDF parsing, Xacro expansion scenarios, and validation features like Inertia calculations.
- **`integration/platforms/blender/`**: Verifies the complete roundtrip process (Import → Scene Setup → Export).

## How to Run Tests

### Standard Python Tests
To run core unit tests and core integration tests:
```bash
pytest tests/unit/core tests/integration/core
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
3. **Roundtrip Integrity**: When adding support for a new URDF tag, always add a corresponding roundtrip test in `integration/platforms/blender/` to ensure export parity.
4. **Mocking**: Use `unittest.mock` to simulate Blender's asynchronous timers or IO operations where possible.
