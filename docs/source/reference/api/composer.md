# Composer — Programmatic Robot Assembly

The Composer is LinkForge's high-level Python API for building and assembling robots
without Blender. It supports two complementary workflows that can be freely combined.

## Overview

```{eval-rst}
.. autoclass:: linkforge_core.composer.robot_assembly.RobotAssembly
   :members:
   :undoc-members:
   :show-inheritance:
   :exclude-members: robot, srdf
```

## Link Builder

The `LinkBuilder` is the fluent interface returned by `RobotAssembly.add_link()`.
It uses a staged pattern — configuration methods accumulate state, and a terminal
method commits the link and joint to the assembly.

```{eval-rst}
.. autoclass:: linkforge_core.composer.robot_assembly.LinkBuilder
   :members:
   :undoc-members:
   :show-inheritance:
```

## Factory Helpers

Convenience constructors for common joint and geometry primitives.

```{eval-rst}
.. automodule:: linkforge_core.composer.factories
   :members:
   :undoc-members:
```

---

## Usage Guide

### Macro-Assembly: Combining Pre-built Robots

Attach a complete robot sub-model (e.g., a gripper) onto a base robot at a
named flange link. The `attach()` method performs a deep copy, so the same
gripper model can be reused across multiple arms.

```python
from pathlib import Path
from linkforge_core.composer.robot_assembly import RobotAssembly
from linkforge_core.parsers import XACROParser

# Load sub-robots from XACRO files
base_arm = XACROParser().parse(Path("ur10e.urdf.xacro"))
gripper  = XACROParser().parse(Path("robotiq_2f140.urdf.xacro"))

# Build assembly
assembly = RobotAssembly("dual_arm_cell", base_arm)
assembly.attach(gripper, at_link="tool0", joint_name="gripper_mount", prefix="left_")
```

### Micro-Construction: The Fluent Builder

Add individual links and joints using the staged builder API:

```python
from linkforge_core.composer.robot_assembly import RobotAssembly
from linkforge_core.models.geometry import Vector3
from linkforge_core.models.joint import JointLimits
from linkforge_core.models import Robot

assembly = RobotAssembly("my_robot", Robot(name="my_robot"))

# Fixed bracket
assembly.add_link("bracket") \
    .with_mass(0.3) \
    .at_origin(xyz=(0.2, 0, 0)) \
    .connect_to("base_link", "bracket_joint") \
    .as_fixed()

# Revolute elbow
assembly.add_link("elbow") \
    .with_mass(1.5) \
    .connect_to("bracket", "elbow_joint") \
    .as_revolute(
        axis=Vector3(0, 0, 1),
        limits=JointLimits(lower=-1.57, upper=1.57, effort=10, velocity=1),
    )

# Wheel (continuous — no position limits)
assembly.add_link("wheel") \
    .with_mass(0.5) \
    .at_origin(xyz=(0, 0.175, 0)) \
    .connect_to("base_link", "wheel_joint") \
    .as_continuous(axis=Vector3(0, 1, 0))
```

### Adding SRDF Semantic Information

After building the kinematic structure, add MoveIt-compatible semantic data
and export both URDF and SRDF in one step:

```python
# Define a planning group
assembly.add_group("arm", base_link="base_link", tip_link="elbow")

# Disable self-collision between adjacent links
assembly.disable_collisions("base_link", "bracket", reason="Adjacent")
assembly.disable_all_collisions(["wheel", "bracket"], reason="Never")

# Export both formats
urdf_str = assembly.export_urdf()
srdf_str = assembly.export_srdf()

with open("my_robot.urdf", "w") as f:
    f.write(urdf_str)
with open("my_robot.srdf", "w") as f:
    f.write(srdf_str)
```

### Builder Method Reference

| Step | Method | Returns | Description |
|---|---|---|---|
| Start | `add_link(name)` | `LinkBuilder` | Creates a new link |
| Config | `.with_mass(value)` | `LinkBuilder` | Sets mass (kg) |
| Config | `.at_origin(xyz, rpy)` | `LinkBuilder` | Sets joint transform |
| Staging | `.connect_to(parent, joint_name)` | `LinkBuilder` | Stages parent/joint |
| Terminal | `.as_fixed()` | `RobotAssembly` | Fixed joint (0 DOF) |
| Terminal | `.as_revolute(axis, limits)` | `RobotAssembly` | Revolute joint (1 DOF) |
| Terminal | `.as_prismatic(axis, limits)` | `RobotAssembly` | Prismatic/sliding joint (1 DOF) |
| Terminal | `.as_continuous(axis)` | `RobotAssembly` | Continuous rotation (no limits) |
| Terminal | `.as_joint(type, ...)` | `RobotAssembly` | Generic escape hatch |

:::{note}
`connect_to()` **must** be called before any terminal method. Calling a
terminal method without staging first raises a `RobotValidationError`.
:::
