# Automated Testing

LinkForge uses a **Three-Tier Testing Strategy** to ensure reliability across its core robotics logic and its Blender integration.

## Test Philosophy

We follow a **Tiered Mocking Model**:
- **Core Tests**: Zero-dependency, pure Python logic.
- **Logic Tests**: Verifies transformation math and adapter logic using mocks. Faster than booting Blender.
- **Integration Tests**: Verifies real `bpy` scene interactions using a headless Blender instance.

### Core Principles
- **Structural Equality**: Every robot model must be compared using `.normalized()` to ensure deterministic verification regardless of internal list ordering.
- **Roundtrip Integrity**: Every supported URDF tag must survive an Import → Edit → Export cycle with bit-for-bit structural identicality.
- **Surgical Cleanup**: We use targeted scene cleanup fixtures (like `blender_clean_scene`) to ensure test isolation without the performance hit of full environment resets.

## Test Organization

The test suite is located in the `tests/` directory:

### 1. Core Tests (`tests/unit/core/`)
High-performance tests for robot models, path-parsing, and kinematics. These have zero external dependencies.

### 2. Platform Logic Tests (`tests/unit/platforms/`)
Tests for adapters, converters, and transformation math. These use **mocks** to simulate the host environment (like Blender), allowing for sub-second execution.

### 3. Integration Tests (`tests/integration/`)
Verifies end-to-end workflows and complex system interactions using real host environments.
- **`parsers/`**: Deep validation of URDF/Xacro parsing.
- **`platforms/blender/`**: Verifies that robots in Blender correctly export back to valid URDF.

## Running the Automated Suite

We use `just` to run specific tiers of the suite:

| Command | Role | Runs in |
|---|---|---|
| `just test-core` | Verifies Robotics Logic | System Python |
| `just test-blender-logic` | Verifies Adapter Logic (Mocked) | System Python |
| `just test-blender` | Verifies Scene Integration | Headless Blender |

### Blender Execution Model
Blender tests use a **two-layer execution model** because Blender ships its own embedded Python interpreter:

1. `blender_launcher.py` (root): Finds Blender and spawns the subprocess.
2. `tests/blender_test_runner.py`: Runs inside Blender, injects paths, and executes `pytest.main()`.

## Continuous Integration (CI)

LinkForge uses GitHub Actions to run the full tiered suite on every Push and Pull Request across Ubuntu, Windows, and macOS.
