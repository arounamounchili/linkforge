# LinkForge Test Suite
**The Safety Net for High-Fidelity Robotics.**

LinkForge uses a **Tiered Testing Architecture** to ensure that robot models remain physically stable and structurally sound across all platforms.

## 📂 Directory Structure

### 1. `unit/`: Component-Level Isolation
- **`unit/core/`**: Tests for IR models, parsers, and physics guardrails (Zero dependencies).
- **`unit/platforms/blender/`**: Tests for platform adapters and translation logic (Uses `mock_bpy_env`).

### 2. `integration/`: End-to-End Fidelity
- **`integration/core/`**: Verifies complex URDF/SRDF round-trips and multi-file XACRO macros.
- **`integration/platforms/blender/`**: Verifies scene manipulation and real Blender export/import cycles.

### 3. Infrastructure
- `../scripts/blender_launcher.py`: Orchestrates tests inside a real (headless) Blender instance.
- `mock_bpy_env.py`: A high-fidelity mock of the Blender API for sub-second logic testing.
- `conftest.py`: Shared fixtures (e.g., `robot_factory`, `examples_dir`).

## 🧪 How to Run Tests

### ⚡ Tier 1: Core Logic
Fast, zero-dependency tests for the heart of LinkForge.
```bash
just test-core
```

### 🚅 Tier 2: Platform Logic (Mocked)
Tests the Blender integration logic without needing to boot Blender.
```bash
just test-blender-logic
```

### 🛰️ Tier 3: Full Integration (Real Blender)
The ultimate fidelity check. Runs tests inside a real Blender environment.
```bash
just test-blender
```

## 📊 Coverage & Quality

To run the full suite and generate a combined HTML coverage report:
```bash
just coverage
```

## 🏗️ Contributor Standards

1. **Physics as Truth**: When adding a new model property, add a unit test in `unit/core/` verifying its physical guardrails.
2. **Round-Trip Fidelity**: Every new feature must include an integration test verifying that data survives a full import-export cycle.
3. **Headless First**: Always try to write a Tier 2 (Mocked) test before resorting to a Tier 3 (Real Blender) test.
