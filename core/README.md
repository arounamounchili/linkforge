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

## 🔭 The LLVM for Robotics

URDF is fragmented, XACRO is XML template hell, and physics engines frequently explode due to bad inertia tensors. LinkForge Core solves this by acting as a **unified intermediate compiler layer** for robot descriptions.

It provides a mathematically pure, zero-dependency Intermediate Representation (IR) with hardened physical validation, scientific inertia solvers, and lossless translation between **URDF**, **XACRO**, and **SRDF**.

---

## ⚡ Why LinkForge Core?

* **⚖️ Physically Guaranteed Sim Stability**: Zero-mass links or unphysical inertia tensors cause simulators like Gazebo or Isaac Sim to crash. LinkForge Core uses the **Mirtich algorithm** (Divergence Theorem) to calculate exact inertia properties from geometries, validated against **Sylvester's Criterion** to ensure physical validity.
* **🔌 Standardized & Namespaced Assembly**: Easily compile complex robots, merge multiple sub-assemblies (e.g. attaching a gripper to an arm), and apply joint prefixing and limits programmatically using the fluent **Composer API**.
* **🛡️ Hardened Sandboxed Security**: Safely parse untrusted third-party robot descriptions. LinkForge Core blocks path-traversal attacks and restrains file reading to designated package boundaries.
* **📦 Light & Portable**: Zero external dependencies. No C++ compilation required, making it highly portable across standard Python environments, CLI tools, and servers.

---

## 🚀 30-Second Quickstart

LinkForge Core exposes a flat, highly curated, and elegant public API. No nested import paths required.

```python
from linkforge.core import RobotBuilder, Box, Vector3, JointLimits

# Initialize a namespaced assembly
assembly = RobotBuilder("forge_arm")

# Create links and chains fluently
assembly.add_link("base_link") \
    .with_mass(5.0) \
    .connect_to("world", "world_joint") \
    .as_fixed()

# geometry parameters automatically calculate exact inertia tensors
assembly.add_link("upper_arm", geometry=Box(size=Vector3(0.1, 0.1, 0.8))) \
    .with_mass(2.5) \
    .connect_to("base_link", "shoulder_yaw") \
    .as_revolute(
        axis=Vector3(0, 0, 1),
        limits=JointLimits(lower=-3.14, upper=3.14, effort=50.0, velocity=2.0)
    )

# Export production-ready, validated XML descriptions
urdf_xml = assembly.export_urdf()
```

---

## 💎 Key Capabilities

### Lossless Ingest & Multi-Phase Linter
Parse existing URDF or XACRO files from the filesystem or memory strings. The parser is completely "lossless" — it preserves unrecognized or custom tags while sanitizing package paths and validating kinematics:

```python
from linkforge.core import read_urdf, validate_robot

# Ingest and auto-resolve package:// bounds
robot = read_urdf("my_robot.urdf")

# Perform kinematic, structural, and physical checks
result = validate_robot(robot)

if result.is_valid:
    print("✓ Robot model is physically and kineamtically sound!")
else:
    for issue in result.errors:
        print(f"  [{issue.code.name}] {issue.message} on {issue.affected_objects}")
```

### Exact Solid-Body Inertia Solver
Compute perfect principal moments of inertia and Center of Mass offsets for primitives or complex triangle meshes, hardened with local origin conditioning to preserve floating-point accuracy:

```python
from linkforge.core import Box, Vector3, calculate_box_inertia

box = Box(size=Vector3(1.0, 0.5, 0.3))
# Automatically computes exact ixx, iyy, izz principal moments
inertia = calculate_box_inertia(box, mass=10.0)
```

---

## 📚 Resources & Documentation

* **📚 Extensive Documentation**: Read the tutorials and how-to guides at [linkforge.readthedocs.io](https://linkforge.readthedocs.io/).
* **🐙 Open Source Repository**: View source, open issues, and join discussions on [GitHub](https://github.com/arounamounchili/linkforge).
* **📄 License**: Standard open-source **[Apache-2.0 License](https://github.com/arounamounchili/linkforge/blob/main/LICENSE)**.
