# LinkForge Core
**The platform-independent Intermediate Representation (IR) and "Robotics Intelligence" engine.**

<p align="center">
  <a href="https://pypi.org/project/linkforge-core/"><img src="https://img.shields.io/pypi/v/linkforge-core.svg?color=blue" alt="PyPI version"></a>
  <a href="https://pypi.org/project/linkforge-core/"><img src="https://img.shields.io/badge/python-3.11+-3776AB" alt="Python versions"></a>
  <a href="https://linkforge.readthedocs.io/"><img src="https://img.shields.io/badge/docs-read%20the%20docs-brightgreen" alt="Documentation Status"></a>
  <a href="https://github.com/arounamounchili/linkforge/blob/main/core/LICENSE"><img src="https://img.shields.io/badge/License-Apache_2.0-blue.svg" alt="License"></a>
</p>

---

## 🔭 What Is LinkForge Core?

Writing and maintaining URDF or SRDF by hand is fragile: inertia values are guessed, collision geometries drift, and physics bugs surface only after a simulator crash — or worse, on hardware. LinkForge Core solves this by treating your robot as **source code with physical constraints**, not a static XML document.

It provides a mathematically rigorous, zero-dependency Intermediate Representation (IR) engine with hardened physical validation, scientific inertia solvers (Mirtich / Sylvester), and lossless round-trip translation between **URDF**, **XACRO**, and **SRDF**.

---

## ⚡ Why LinkForge Core?

* **⚖️ Physically Guaranteed Sim Stability**: Zero-mass links or unphysical inertia tensors cause simulators like Gazebo or Isaac Sim to crash. LinkForge Core uses the **Mirtich algorithm** (Divergence Theorem) to calculate exact inertia properties from geometries, validated against **Sylvester's Criterion** to ensure physical validity.
* **🔌 Standardized & Namespaced Assembly**: Easily compile complex robots, merge multiple sub-assemblies (e.g. attaching a gripper to an arm), and apply joint prefixing and limits programmatically using the fluent **Composer API**.
* **🛡️ Hardened Sandboxed Security**: Safely parse untrusted third-party robot descriptions. LinkForge Core blocks path-traversal attacks and restrains file reading to designated package boundaries.
* **📦 Light & Portable**: Zero external dependencies. No C++ compilation required, making it highly portable across standard Python environments, CLI tools, and servers.

---

## 🚀 Quickstart

LinkForge Core exposes a flat, highly curated, and elegant public API. No nested import paths required.

```python
from linkforge.core import RobotBuilder, box, cylinder

# Initialize the assembly builder
builder = RobotBuilder("forge_arm")

# Define the base link (root of the robot)
builder.link("base_link") \
    .visual(box(0.2, 0.2, 0.1)) \
    .collision() \
    .mass(5.0) \
    .root()

# Define and connect the upper arm link via a revolute joint
builder.link("upper_arm", parent="base_link") \
    .visual(cylinder(0.05, 0.8)) \
    .collision() \
    .mass(2.5) \
    .revolute(
        axis=(0, 0, 1),
        limits=(-3.14, 3.14),
        effort=50.0,
        velocity=2.0
    ) \
    .commit()

# Export production-ready, validated URDF XML string
urdf_xml = builder.export_urdf()
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
    print("✓ Robot model is physically and kinematically sound!")
else:
    for issue in result.errors:
        print(f"  [{issue.code.name}] {issue.message} on {issue.affected_objects}")
```

### Exact Solid-Body Inertia Solver
Compute perfect principal moments of inertia and Center of Mass offsets for primitives or complex triangle meshes, hardened with local origin conditioning to preserve floating-point accuracy:

```python
from linkforge.core import Box, Vector3, calculate_inertia

box = Box(size=Vector3(1.0, 0.5, 0.3))
# Automatically computes exact ixx, iyy, izz principal moments
inertia = calculate_inertia(box, mass=10.0)
```

---

## 📚 Resources & Documentation

* **📚 Extensive Documentation**: Read the tutorials and how-to guides at [linkforge.readthedocs.io](https://linkforge.readthedocs.io/).
* **🐙 Open Source Repository**: View source, open issues, and join discussions on [GitHub](https://github.com/arounamounchili/linkforge).
* **📄 License**: Standard open-source **[Apache-2.0 License](https://github.com/arounamounchili/linkforge/blob/main/core/LICENSE)**.
