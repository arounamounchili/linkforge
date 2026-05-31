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
exceptions
```

## Overview

LinkForge is organized into two main layers:

### Core Layer (`linkforge.core`)

Platform-independent robot modeling and URDF/XACRO processing.

- **Composer**: `RobotBuilder` and `LinkBuilder` — the programmatic robot building API
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

The `RobotBuilder` Composer is the recommended way to build robots in Python.
It handles validation, prefixing, and SRDF generation automatically.

```python
from linkforge.core import RobotBuilder, box, cylinder

builder = RobotBuilder("my_robot")

builder.link("base_link").visual(box(0.5, 0.5, 0.1)).mass(5.0).root()
builder.link("arm", parent="base_link") \
    .visual(cylinder(0.05, 0.5)) \
    .mass(2.0) \
    .revolute(axis=(0, 0, 1), limits=(-1.57, 1.57), effort=10, velocity=1) \
    .commit()

urdf = builder.export_urdf()
```

See the [Composer reference](composer) for the full API, including macro-assembly
and SRDF export.

### Parsing URDF

```python
from linkforge.core import URDFParser
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
from linkforge.core import Box, Cylinder, Vector3, calculate_inertia

# Box inertia
box = Box(size=Vector3(1.0, 0.5, 0.3))
inertia = calculate_inertia(box, mass=10.0)

# Cylinder inertia
cylinder = Cylinder(radius=0.1, length=0.5)
inertia = calculate_inertia(cylinder, mass=5.0)
```

### Validation

```python
from linkforge.core import validate_robot

# Validate robot
result = validate_robot(robot)

if result.is_valid:
    print("✓ Robot is valid!")
else:
    print("✗ Validation errors:")
    for error in result.errors:
        print(f"  - {error.message}")
```

## Module Index

- {ref}`modindex`
- {ref}`genindex`
