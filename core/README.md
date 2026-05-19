# LinkForge Core
**The platform-independent Intermediate Representation (IR) and "Robotics Intelligence" engine.**

<p align="center">
  <a href="https://pypi.org/project/linkforge-core/"><img src="https://img.shields.io/pypi/v/linkforge-core.svg?color=blue" alt="PyPI version"></a>
  <a href="https://pypi.org/project/linkforge-core/"><img src="https://img.shields.io/pypi/pyversions/linkforge-core.svg" alt="Python versions"></a>
  <a href="https://linkforge.readthedocs.io/"><img src="https://img.shields.io/badge/docs-read%20the%20docs-brightgreen" alt="Documentation Status"></a>
  <a href="https://github.com/arounamounchili/linkforge/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-Apache_2.0-blue.svg" alt="License"></a>
  <a href="https://github.com/astral-sh/ruff"><img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json" alt="Ruff"></a>
</p>

---

LinkForge Core serves as the **"LLVM for Robotics."** It decouples the mathematical and physical description of a robot from external design tools (like Blender, FreeCAD, or Onshape). It provides a mathematically pure, zero-dependency Intermediate Representation (IR) with hardened validation, scientific inertia solvers, and lossless translation between **URDF**, **XACRO**, and **SRDF**.

---

## ✨ Features at a Glance

* **Zero-Dependency Core**: Lightweight and highly portable. No complex C++ bindings or NumPy dependency for simulation logic.
* **Unified Intermediate Representation (IR)**: A high-fidelity object model representing links, kinematic joints, sensors, actuators, and transmissions.
* **Production-Grade Physics Hardening**: Bounded numerical integration using the **Mirtich algorithm** (Divergence Theorem) with local origin-shifting to avoid floating-point loss, physicality audits via **Sylvester’s Criterion**, and topology checks.
* **Modular Validation Pipeline**: A compiler-like linter registry that executes kinematic, physical, and semantic invariants, catching bugs before they hit Gazebo or MoveIt.
* **Lossless Parsing & Generation**: Gracefully parses and serializes XML, retaining custom and unrecognized tags while sanitizing package boundaries.
* **Composer API**: Assemble complex kinematic chains, merge sub-assemblies, apply joint namespaces, and configure self-collision matrices programmatically.

---

## 🚀 The Core Code Tour

LinkForge Core exposes a flat, highly curated, and elegant public API. No nested submodule imports required.

### 1. Programmatic Assembly (The Composer API)
The `RobotBuilder` Composer is the recommended way to model robots programmatically. It automatically computes solid-body mass dynamics, applies kinematic links, and configures joints in a single fluent interface.

```python
from linkforge.core import RobotBuilder, Box, Vector3, JointLimits

# Initialize the assembly
assembly = RobotBuilder("forge_arm")

# Create links and chains fluently
assembly.add_link("base_link") \
    .with_mass(5.0) \
    .connect_to("world", "world_joint") \
    .as_fixed()

assembly.add_link("upper_arm", geometry=Box(size=Vector3(0.1, 0.1, 0.8))) \
    .with_mass(2.5) \
    .connect_to("base_link", "shoulder_yaw") \
    .as_revolute(
        axis=Vector3(0, 0, 1),
        limits=JointLimits(lower=-3.14, upper=3.14, effort=50.0, velocity=2.0)
    )

# Export complete, structured XML representations directly
urdf_xml = assembly.export_urdf()
```

### 2. Lossless Ingest & Multi-Phase Validation
Ingest URDFs or XACRO files from the filesystem or memory strings, and run deep linter checks:

```python
from linkforge.core import read_urdf, validate_robot

# Lossless parsing (preserves custom tags, formats XML safely)
robot = read_urdf("my_robot.urdf")

# Perform full multi-phase structural, kinematic, and physical validation
result = validate_robot(robot)

if result.is_valid:
    print("✓ Robot model is physically and structurally sound!")
else:
    print("✗ Validation failed:")
    for issue in result.errors:
        print(f"  [{issue.code.name}] {issue.message} on {issue.affected_objects}")
```

### 3. Scientific Mass Properties & Inertia Solvers
Calculate exact moments of inertia for standard primitives or complex triangle meshes, hardened with local origin conditioning and Sylvester criteria verification to ensure positive semi-definiteness:

```python
from linkforge.core import Box, Cylinder, Vector3, calculate_box_inertia

box = Box(size=Vector3(1.0, 0.5, 0.3))
# Automatically computes exact moments of inertia: ixx, iyy, izz
inertia = calculate_box_inertia(box, mass=10.0)
print(f"Computed mass properties tensor: {inertia}")
```

---

## 📂 Architecture: Hexagonal Philosophy

LinkForge Core acts as the pure domain kernel under a **Ports & Adapters (Hexagonal)** architectural pattern:

```
        ┌──────────────────────────────────────────────────────────┐
        │                    Platform Adapters                     │
        │      (Blender Extension, FreeCAD Macros, ROS 2 Node)     │
        └────────────────────────────┬─────────────────────────────┘
                                     │
                                     ▼
                     ┌──────────────────────────────┐
                     │     LinkForge Public API     │
                     │          (io.py)             │
                     └───────────────┬──────────────┘
                                     │
         ┌───────────────────────────┼───────────────────────────┐
         ▼                           ▼                           ▼
 ┌───────────────┐           ┌───────────────┐           ┌───────────────┐
 │   Models IR   │           │    Physics    │           │  Validation   │
 │ (robot/joint) │           │ (Mirtich/COM) │           │ (linter/sec)  │
 └───────────────┘           └───────────────┘           └───────────────┘
```

---

## 🛠️ Local Development & Testing

LinkForge Core uses [`uv`](https://docs.astral.sh/uv/) for high-speed package management and [`just`](https://github.com/casey/just) for streamlined multi-platform execution commands.

```bash
# Clone the repository
git clone https://github.com/arounamounchili/linkforge.git
cd linkforge

# Setup a clean development environment and sync dependencies
just install

# Run the complete test suite (achieves 99% branch coverage)
just test-unit-core

# Run Ruff linter and MyPy type checks
just check
```

---

## 👥 Community & Contributing

We welcome issues, feedback, and contributions!
* **Found a bug?** Let us know in our [GitHub Issue Tracker](https://github.com/arounamounchili/linkforge/issues).
* **Have ideas?** Join our [GitHub Discussions](https://github.com/arounamounchili/linkforge/discussions).
* **License**: LinkForge is open-source software licensed under the **[Apache-2.0 License](https://github.com/arounamounchili/linkforge/blob/main/LICENSE)**.
