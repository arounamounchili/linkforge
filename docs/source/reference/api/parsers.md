# Parsers

URDF and XACRO parsers for converting files to Python objects.

## URDF Parser

### URDFParser Class

```{eval-rst}
.. autoclass:: linkforge_core.parsers.urdf_parser.URDFParser
   :members:
   :undoc-members:
   :show-inheritance:
```

### Component Parsing Helpers

```{eval-rst}
.. autofunction:: linkforge_core.parsers.urdf_parser.parse_link

.. autofunction:: linkforge_core.parsers.urdf_parser.parse_joint

.. autofunction:: linkforge_core.parsers.urdf_parser.parse_sensor_from_gazebo
```

## Usage Examples

### Parse URDF File

```python
from linkforge_core.parsers import URDFParser
from pathlib import Path

robot = URDFParser().parse(Path("my_robot.urdf"))
print(f"Loaded robot: {robot.name}")
```

### Parse URDF String

```python
from linkforge_core.parsers import URDFParser

urdf_content = """<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link"/>
</robot>"""

robot = URDFParser().parse_string(urdf_content)
```

### Robustness & Security

The parser includes professional-grade protections for production robotics:

*   **Duplicate Name Resolution**: Re-names conflicting link/joint names (e.g., `link_duplicate_1`) to preserve kinematic tree integrity while alerting the user.
*   **DoS Protection**: Enforces a maximum XML depth (100 levels) and file size (100 MB) to prevent "XML Bomb" attacks.
*   **Path Sandboxing**: Validates all mesh paths to prevent directory traversal and ensure assets remain within authorized project folders.
*   **Resilient Skip**: Malformed geometry or broken joint references are logged as warnings, allowing the rest of the robot to load successfully.
