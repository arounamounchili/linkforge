# Generators

URDF and XACRO generators for converting Python objects to files.

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
# - robot.xacro (main file)
# - properties.xacro (parameters)
# - macros.xacro (reusable macros)
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
