# API Reference

Complete API documentation for LinkForge.

```{toctree}
:maxdepth: 2

models
parsers
generators
physics
validation
blender
```

## Overview

LinkForge is organized into two main layers:

### Core Layer (`linkforge.core`)

Platform-independent robot modeling and URDF/XACRO processing.

- **Models**: Data structures (`Robot`, `Link`, `Joint`, `Sensor`, etc.)
- **Parsers**: URDF/XACRO → Python objects
- **Generators**: Python objects → URDF/XACRO
- **Physics**: Inertia calculations
- **Validation**: Error checking and security

### Blender Layer (`linkforge.blender`)

Blender-specific integration.

- **Operators**: User actions (export, create, etc.)
- **Panels**: UI panels
- **Properties**: Blender scene properties
- **Utils**: Blender-specific utilities

## Quick Reference

### Creating a Robot Programmatically

```python
from linkforge.core.models import Robot, Link, Joint, JointType, Inertial, InertiaTensor
from linkforge.core import URDFGenerator

# Create robot
robot = Robot(
    name="my_robot",
    links=[
        Link(
            name="base_link",
            inertial=Inertial(
                mass=10.0,
                inertia=InertiaTensor(ixx=1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=1.0)
            )
        ),
        Link(name="link1", inertial=...)
    ],
    joints=[
        Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="base_link",
            child="link1"
        )
    ]
)

# Generate URDF
generator = URDFGenerator()
urdf_string = generator.generate(robot)

# Save to file
with open("robot.urdf", "w") as f:
    f.write(urdf_string)
```

### Parsing URDF

```python
from linkforge.core.parsers.urdf_parser import parse_urdf
from pathlib import Path

# Parse URDF file
robot = parse_urdf(Path("robot.urdf"))

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
from linkforge.core.physics.inertia import calculate_box_inertia, calculate_cylinder_inertia
from linkforge.core.models.geometry import Box, Cylinder, Vector3

# Box inertia
box = Box(size=Vector3(1.0, 0.5, 0.3))
inertia = calculate_box_inertia(box, mass=10.0)

# Cylinder inertia
cylinder = Cylinder(radius=0.1, length=0.5)
inertia = calculate_cylinder_inertia(cylinder, mass=5.0)
```

### Validation

```python
from linkforge.core.validation.validator import RobotValidator

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
