# Generators

URDF, XACRO, and SRDF generators for converting Python objects to files.

## Base XML Engine

All XML-based generators in LinkForge inherit from a shared base engine that handles common geometry, inertial, and origin logic.

```{eval-rst}
.. autoclass:: linkforge_core.generators.xml_base.RobotXMLGenerator
   :members:
   :undoc-members:
   :show-inheritance:
```

## URDF Generator

```{eval-rst}
.. autoclass:: linkforge_core.generators.urdf_generator.URDFGenerator
   :members:
   :undoc-members:
   :show-inheritance:
```

## XACRO Generator

```{eval-rst}
.. autoclass:: linkforge_core.generators.xacro_generator.XACROGenerator
   :members:
   :undoc-members:
   :show-inheritance:
```

## SRDF Generator

The SRDF generator is documented with the rest of the SRDF layer (models, parser, generator)
on the dedicated [SRDF reference page](srdf.md).

## Usage Examples

### Generate URDF

```python
from linkforge_core.models import Robot, Link, Inertial, InertiaTensor
from linkforge_core import URDFGenerator

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
        )
    ]
)

# Generate URDF
generator = URDFGenerator(pretty_print=True)
urdf_string = generator.generate(robot)

# Save to file
with open("robot.urdf", "w") as f:
    f.write(urdf_string)
```

### Generate XACRO

```python
from linkforge_core import XACROGenerator
from pathlib import Path

# Generate XACRO with split files
generator = XACROGenerator(
    split_files=True,
    output_dir=Path("robot_description")
)

files = generator.generate(robot)
# Creates:
# - robot.xacro (main assembly)
# - robot_properties.xacro (materials & dimensions)
# - robot_macros.xacro (reusable geometries)
# - robot_ros2_control.xacro (hardware plugins & interfaces)
```

### Export Options

```python
# Compact output (no pretty printing)
generator = URDFGenerator(pretty_print=False)
compact_urdf = generator.generate(robot)

# With custom URDF path (for relative mesh paths)
generator = URDFGenerator(urdf_path=Path("robots/my_robot.urdf"))
urdf = generator.generate(robot)
# Mesh paths will be relative to robots/ directory
```

### Round-Trip Verification

```python
from linkforge_core.parsers import URDFParser

# Original robot
robot1 = create_robot()

# Export to URDF
generator = URDFGenerator()
urdf = generator.generate(robot1)

# Re-import
robot2 = URDFParser().parse_string(urdf)

# Verify fidelity
assert robot2.name == robot1.name
assert len(robot2.links) == len(robot1.links)
assert len(robot2.joints) == len(robot1.joints)
```
