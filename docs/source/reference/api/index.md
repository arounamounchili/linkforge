# API Reference

Complete API documentation for LinkForge.

```{toctree}
:maxdepth: 2

composer
models
srdf
parsers
generators
physics
validation
checks
graph
blender
```

## Overview

LinkForge is organized into two main layers:

### Core Layer (`linkforge_core`)

Platform-independent robot modeling and URDF/XACRO processing.

- **Composer**: `RobotAssembly` and `LinkBuilder` — the programmatic robot building API
- **Models**: Data structures (`Robot`, `Link`, `Joint`, `Sensor`, `SemanticRobotDescription`, etc.)
- **Parsers**: URDF/XACRO/SRDF → Python objects
- **Generators**: Python objects → URDF/XACRO/SRDF
- **Physics**: Inertia calculations
- **Validation**: Modular check registry and security
- **Graph**: Kinematic tree utilities

### Blender Layer (`linkforge.blender`)

Blender-specific integration.

- **Operators**: User actions (export, create, etc.)
- **Panels**: UI panels
- **Properties**: Blender scene properties
- **Utils**: Blender-specific utilities

## Quick Reference

### Building a Robot Programmatically

The `RobotAssembly` Composer is the recommended way to build robots in Python.
It handles validation, prefixing, and SRDF generation automatically.

```python
from linkforge_core.composer.robot_assembly import RobotAssembly
from linkforge_core.models import Robot
from linkforge_core.models.geometry import Vector3
from linkforge_core.models.joint import JointLimits

assembly = RobotAssembly("my_robot", Robot(name="my_robot"))

assembly.add_link("base_link").with_mass(5.0).connect_to("world", "world_joint").as_fixed()
assembly.add_link("arm").with_mass(2.0).connect_to("base_link", "shoulder").as_revolute(
    axis=Vector3(0, 0, 1),
    limits=JointLimits(lower=-1.57, upper=1.57, effort=10, velocity=1),
)

urdf = assembly.export_urdf()
```

See the [Composer reference](composer) for the full API, including macro-assembly
and SRDF export.

### Parsing URDF

```python
from linkforge_core.parsers import URDFParser
from pathlib import Path

# Parse URDF file
robot = URDFParser().parse(Path("robot.urdf"))

# Access components
print(f"Robot: {robot.name}")
print(f"Links: {len(robot.links)}")
print(f"Joints: {len(robot.joints)}")

# Iterate over links
for link in robot.links:
    print(f"  Link: {link.name}, Mass: {link.inertial.mass if link.inertial else 'N/A'}")
```

### Calculating Inertia

```python
from linkforge_core.physics.inertia import calculate_box_inertia, calculate_cylinder_inertia
from linkforge_core.models.geometry import Box, Cylinder, Vector3

# Box inertia
box = Box(size=Vector3(1.0, 0.5, 0.3))
inertia = calculate_box_inertia(box, mass=10.0)

# Cylinder inertia
cylinder = Cylinder(radius=0.1, length=0.5)
inertia = calculate_cylinder_inertia(cylinder, mass=5.0)
```

### Validation

```python
from linkforge_core.validation.validator import RobotValidator

# Validate robot
validator = RobotValidator()
result = validator.validate(robot)

if result.is_valid:
    print("✓ Robot is valid!")
else:
    print("✗ Validation errors:")
    for error in result.errors:
        print(f"  - {error}")
```

## Module Index

- {ref}`modindex`
- {ref}`genindex`
