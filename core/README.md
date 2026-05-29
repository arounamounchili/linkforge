# LinkForge Core
**The platform-independent Intermediate Representation (IR) engine for robot descriptions.**

<p align="center">
  <a href="https://pypi.org/project/linkforge-core/"><img src="https://img.shields.io/pypi/v/linkforge-core.svg?color=blue" alt="PyPI version"></a>
  <a href="https://pypi.org/project/linkforge-core/"><img src="https://img.shields.io/badge/python-3.11+-3776AB" alt="Python versions"></a>
  <a href="https://linkforge.readthedocs.io/"><img src="https://img.shields.io/badge/docs-read%20the%20docs-brightgreen" alt="Documentation Status"></a>
  <a href="https://github.com/arounamounchili/linkforge/blob/main/core/LICENSE"><img src="https://img.shields.io/badge/License-Apache_2.0-blue.svg" alt="License"></a>
</p>

---

## 🔭 What Is LinkForge Core?

Writing and maintaining URDF or SRDF by hand is fragile: inertia values are guessed, collision geometries drift, and physics bugs surface only after a simulator crash (or worse, on hardware). LinkForge Core solves this by treating your robot as **source code with physical constraints**, not a static XML document.

It provides a mathematically rigorous, zero-dependency Intermediate Representation (IR) engine with hardened physical validation, scientific inertia solvers (Mirtich / Sylvester), and lossless round-trip translation between **URDF**, **XACRO**, and **SRDF**.

---

## 📦 Installation

```bash
pip install linkforge-core
```

Zero external dependencies. No Blender, no ROS installation, no C++ compilation required.

---

## ⚡ Why LinkForge Core?

- **⚖️ Physically Guaranteed Sim Stability**: Zero-mass links or unphysical inertia tensors cause simulators like Gazebo or Isaac Sim to crash. LinkForge Core uses the **Mirtich algorithm** (Divergence Theorem) to calculate exact inertia properties from geometries, validated against **Sylvester's Criterion** to ensure physical validity.
- **🔌 Standardized & Namespaced Assembly**: Easily compile complex robots, merge multiple sub-assemblies (e.g. attaching a gripper to an arm), and apply joint prefixing and limits programmatically using the fluent **Composer API**.
- **🛡️ Hardened Sandboxed Security**: Safely parse untrusted third-party robot descriptions. LinkForge Core blocks path-traversal attacks and restrains file reading to designated package boundaries.
- **📦 Light & Portable**: Zero external dependencies. No C++ compilation required, making it highly portable across standard Python environments, CI/CD pipelines, and HPC clusters.

---

## 🚀 Quickstart

LinkForge Core exposes a flat, curated public API. No nested import paths required.

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

# Compile to production-ready, validated URDF XML
urdf_xml = builder.export_urdf()
```

---

## 💎 Key Capabilities

### Parse, Validate & Compile (Full Round-Trip)

Ingest existing URDF, XACRO, or SRDF files into the IR, validate them, and compile back out to any target format without data loss:

```python
from linkforge.core import read_urdf, validate_robot, write_urdf, write_xacro

# Ingest and auto-resolve package:// paths
robot = read_urdf("my_robot.urdf")

# Perform kinematic, structural, and physical checks
result = validate_robot(robot)

if result.is_valid:
    # Compile back to URDF or XACRO
    write_urdf(robot, "my_robot_validated.urdf")
    write_xacro(robot, "my_robot.xacro")
else:
    for issue in result.errors:
        print(f"  [{issue.code.name}] {issue.message} on {issue.affected_objects}")
```

---

### MoveIt 2 & SRDF Semantic Composition

Programmatically compose MoveIt 2 planning groups, end-effectors, and self-collision matrices for SRDF without manually editing XML:

```python
from linkforge.core import RobotBuilder, read_srdf, write_srdf

builder = RobotBuilder("my_arm")
# ... define links and joints ...

# Add a kinematic planning group via base/tip chain shorthand
builder.semantic.group(
    "arm",
    base_link="base_link",
    tip_link="wrist_link",
)

# Define an end-effector
builder.semantic.end_effector(
    name="gripper",
    group="gripper_group",
    parent_link="wrist_link",
    parent_group="arm",
)

# Export both URDF + SRDF in one pass
urdf_xml = builder.export_urdf()
srdf_xml = builder.export_srdf()

# Or parse and re-export an existing SRDF
semantic = read_srdf("my_robot.srdf")
write_srdf(semantic, "my_robot_updated.srdf")
```

---

### Exact Solid-Body Inertia Solver

Compute principal moments of inertia and Center of Mass offsets for primitives or complex triangle meshes, hardened with local origin conditioning for floating-point accuracy:

```python
from linkforge.core import Box, Vector3, calculate_inertia

geometry = Box(size=Vector3(1.0, 0.5, 0.3))
# Automatically computes exact ixx, iyy, izz principal moments
inertia = calculate_inertia(geometry, mass=10.0)
```

---

### Sensor Suite

Define cameras, LiDAR, IMU, GPS, and force/torque sensors with configurable noise models directly in the IR:

```python
from linkforge.core import RobotBuilder

builder = RobotBuilder("sensor_bot")
# ... define links ...

# Add a LiDAR sensor to a link
builder.link("lidar_link", parent="base_link") \
    .lidar("front_lidar", range_min=0.1, range_max=30.0, samples=360) \
    .commit()

# Add a camera sensor
builder.link("camera_link", parent="base_link") \
    .camera("front_camera", width=1280, height=720) \
    .commit()
```

---

### Headless Use in CI / ML Pipelines

`linkforge-core` has zero GUI dependencies, making it ideal for headless environments:

```python
# ci_validate.py — run in any CI/CD pipeline or HPC cluster
from linkforge.core import read_urdf, validate_robot
import sys

robot = read_urdf("robot.urdf")
result = validate_robot(robot)

if not result.is_valid:
    for issue in result.errors:
        print(f"ERROR: [{issue.code.name}] {issue.message}")
    sys.exit(1)

print("✓ Robot model validated successfully.")
```

```bash
# In your CI pipeline:
pip install linkforge-core
python ci_validate.py
```

---

## 📚 Resources & Documentation

- **📚 Extensive Documentation**: Read the tutorials and how-to guides at [linkforge.readthedocs.io](https://linkforge.readthedocs.io/).
- **🐙 Open Source Repository**: View source, open issues, and join discussions on [GitHub](https://github.com/arounamounchili/linkforge).
- **📄 License**: Standard open-source **[Apache-2.0 License](https://github.com/arounamounchili/linkforge/blob/main/core/LICENSE)**.
