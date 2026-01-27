# Parsers

URDF and XACRO parsers for converting files to Python objects.

## URDF Parser

### Main Functions

```{eval-rst}
.. autofunction:: linkforge_core.parsers.urdf_parser.parse_urdf

.. autofunction:: linkforge_core.parsers.urdf_parser.parse_urdf_string
```

### Geometry Parsing

```{eval-rst}
.. autofunction:: linkforge_core.parsers.urdf_parser.parse_geometry

.. autofunction:: linkforge_core.parsers.urdf_parser.parse_origin
```

### Component Parsing

```{eval-rst}
.. autofunction:: linkforge_core.parsers.urdf_parser.parse_link

.. autofunction:: linkforge_core.parsers.urdf_parser.parse_joint

.. autofunction:: linkforge_core.parsers.urdf_parser.parse_sensor_from_gazebo
```

## Usage Examples

### Parse URDF File

```python
from linkforge_core.parsers.urdf_parser import parse_urdf
from pathlib import Path

robot = parse_urdf(Path("my_robot.urdf"))
print(f"Loaded robot: {robot.name}")
print(f"Links: {len(robot.links)}")
print(f"Joints: {len(robot.joints)}")
```

### Parse URDF String

```python
from linkforge_core.parsers.urdf_parser import parse_urdf_string

urdf_content = """
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link"/>
</robot>
"""

robot = parse_urdf_string(urdf_content)
```

### Error Handling

The parser is resilient and logs warnings instead of crashing:

```python
# Invalid geometry is skipped with warning
urdf_with_errors = """
<robot name="test">
  <link name="link1">
    <visual>
      <geometry>
        <box size="-1 2 3"/>  <!-- Invalid: negative dimension -->
      </geometry>
    </visual>
  </link>
</robot>
"""

robot = parse_urdf_string(urdf_with_errors)
# Warning logged: "Invalid box geometry ignored"
# Robot still created, but visual is skipped
assert len(robot.links[0].visuals) == 0
```
