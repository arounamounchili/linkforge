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


## XACRO Parser

### XACROParser Class

The `XACROParser` provides native, pure-Python resolution of XACRO files. It handles macros, properties, math expressions, and conditional blocks without external ROS dependencies.

```{eval-rst}
.. autoclass:: linkforge_core.parsers.xacro_parser.XACROParser
   :members:
   :undoc-members:
   :show-inheritance:
```

### XacroResolver Class (Internal)

The internal engine used by `XACROParser` for hierarchical property resolution and macro substitution.

```{eval-rst}
.. autoclass:: linkforge_core.parsers.xacro_parser.XacroResolver
   :members:
   :undoc-members:
```

## Usage Examples

### Parse XACRO File

```python
from linkforge_core.parsers import XACROParser
from pathlib import Path

# Natively resolves properties, macros, and includes
robot = XACROParser().parse(Path("robot.urdf.xacro"))
print(f"Resolved robot: {robot.name}")
```

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
*   **XACRO Debugging Support**: Natively evaluates and routes `xacro.warning()`, `xacro.error()`, `xacro.fatal()`, and `xacro.message()` calls to the LinkForge Python logger, allowing users to see in-file debug messages directly in the console.
*   **Resilient Skip**: Malformed geometry or broken joint references are logged as warnings, allowing the rest of the robot to load successfully.
