# LinkForge Test Suite

LinkForge uses `pytest` to test both the core physics/kinematics engine and the Blender platform integration.

## Directory Structure

The test suite separates pure logic from platform-specific behavior:

### 1. `unit/`
Isolated tests for individual components avoiding external dependencies.
- **`unit/core/`**: Tests for robot models, parsers, and physics utilities.
- **`unit/platforms/blender/`**: Tests for Blender utilities (scene helpers, visualization).

### 2. `integration/`
End-to-end tests verifying the interaction between multiple components.
- **`integration/core/`**: Verifies complex URDF/SRDF parsing and validation.
- **`integration/platforms/blender/`**: Verifies the complete roundtrip process.

### 3. Infrastructure
- `blender_launcher.py` (Root): CLI tool to run tests inside a Blender environment.
- `mock_bpy_env.py`: Comprehensive mock of the Blender API for fast logic testing.
- `conftest.py`, `core_test_utils.py`, `blender_test_utils.py`: Shared fixtures and assertions.

## How to Run Tests

### Standard Python Tests
To run core unit tests and core integration tests:
```bash
just test-core
```

### Blender-Dependent Tests
#### A. Fast Logic Testing (No Blender Required)
Tests the Blender integration logic using a comprehensive mock environment. This is very fast and runs in standard Python.
```bash
just test-blender-logic
```

#### B. Full Integration Testing (Requires Real Blender)
Runs tests inside a real Blender instance to verify UI, visualization, and roundtrip fidelity.
```bash
just test-blender
```
*Note: Ensure your `BLENDER_PATH` environment variable is set or Blender is installed at its default location.*

## 📊 Test Coverage

To run the entire suite and generate a combined coverage report (HTML):
```bash
just coverage
```
The report will be available at `htmlcov/index.html`.

## Best Practices for Contributors

1. **Use Fixtures**: Place shared test resources (mock robots, custom builders) in `tests/conftest.py`. Prefer self-contained tests (inline strings or programmatic construction) over external file dependencies.
2. **Platform Isolation**: If a test doesn't explicitly need a 3D viewport or `bpy` data structures, place it in `core`.
3. **Roundtrip Integrity**: When adding support for a new URDF tag, always add a corresponding roundtrip test in `integration/platforms/blender/` to ensure export parity.
4. **Mocking**: Use `unittest.mock` to simulate Blender's asynchronous timers or IO operations where possible.
