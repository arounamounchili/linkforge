# LinkForge Core
**The Universal Intermediate Representation (IR) for Robotics.**

LinkForge Core is the platform-independent "Robotics Intelligence" engine. It serves as the **LLVM for Robotics**, providing a mathematically pure, high-fidelity IR for parsing, generating, and validating robot descriptions across any platform.

## 🚀 The "Front Door" API

LinkForge Core provides a clean, centralized, and curated API designed for high-fidelity robotics workflows.

```python
import linkforge.core as lf

# 1. Ingest
robot = lf.read_urdf("my_robot.urdf")

# 2. Hardened Validation (Physics & Kinematics)
result = lf.validate_robot(robot)
if not result.is_valid:
    print(f"Danger! {result.errors[0].message}")

# 3. Deploy
lf.write_xacro(robot, "hardened_robot.xacro")
```

## 🏗️ Architectural Pillars

- **Fidelity-First Models**: High-precision data structures for Links, Joints, Sensors, and Transmissions.
- **Physics Hardening**: Built-in guardrails for mass properties and numerical stability.
- **Bi-directional IO**: Lossless translation between URDF, XACRO, and SRDF.
- **Composer API**: Programmatic robot assembly and namespaced merging.
- **Modular Validation**: A registry-based linter that catches simulation-breaking errors early.

## 📂 Internal Structure

- `src/linkforge/core/`: The heart of the IR.
  - `models/`: Entity representations (Robot, Link, Sensor, etc.).
  - `composer/`: Modular assembly and factory patterns.
  - `physics/`: Scientific inertia and stability guardrails.
  - `validation/`: The "Linter for Robotics" check suite.
  - `parsers/` & `generators/`: High-fidelity XML translation.
  - `io.py`: Functional entry points for quick tasks.

## 🛠️ Development

Managed with [`uv`](https://docs.astral.sh/uv/) and [`just`](https://github.com/casey/just).

```bash
just install   # Setup dev environment
just test-core # Run core suite
just check     # Run lint & type checks
```
