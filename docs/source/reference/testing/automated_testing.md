# Automated Testing

LinkForge uses a multi-layered automated testing strategy to ensure reliability across its core robotics logic and its Blender integration.

## Test Philosophy

We follow the principle of **"Real Data over Mocks"**.
- **Blender Tests**: We run a real (headless) instance of Blender via the `bpy` API rather than mocking the graphics engine.
- **Roundtrip Integrity**: Every supported URDF tag must survive an Import → Edit → Export cycle with 100% numerical precision.
- **100% Core Coverage**: All fundamental robotics logic in `linkforge_core` (models, parsers, and utilities) is verified with 100% unit test coverage.

## Test Organization

The test suite is located in the `tests/` directory and is organized into two primary pillars:

### 1. Unit Tests (`tests/unit/`)
Tests individual components in isolation.
- **`unit/core/`**: High-performance tests for robot models, path-parsing, and kinematics. These have zero external dependencies.
- **`unit/blender/`**: Tests for UI elements, scene converters, and custom gizmos.

### 2. Integration Tests (`tests/integration/`)
Verifies end-to-end workflows and complex system interactions.
- **`parsers/`**: Deep validation of URDF/Xacro parsing, including nested macros and includes.
- **`blender/`**: Verifies that robots imported into Blender correctly export back to valid URDF.
- **`features/`**: Validation of high-level features like automated inertia calculation and `ros2_control` wiring.

## Running the Automated Suite

### Core & Parser Tests
These can be run using any standard `pytest` environment:
```bash
just test-core
# Or manually: pytest tests/unit/core
```

### Blender Headless Tests
To verify Blender-specific behavior (viewports, mesh exports, properties):
```bash
# MacOS / Linux
just test-blender
```

## Continuous Integration (CI)

LinkForge uses GitHub Actions to run the full test suite on every Push and Pull Request across:
- **Ubuntu** (Primary Linux environment)
- **Windows** (Varying file path handling)
- **macOS** (Metal/OpenGL compatibility checks)

See our [GitHub Actions](https://github.com/arounamounchili/linkforge/actions) for real-time status.
