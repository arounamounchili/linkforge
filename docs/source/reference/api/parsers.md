# Parsers

URDF, XACRO, and SRDF parsers for converting files into Python objects.

## URDF Parser

### URDFParser Class

```{eval-rst}
.. autoclass:: linkforge.core.parsers.urdf_parser.URDFParser
   :members:
   :undoc-members:
   :show-inheritance:
```


## XACRO Parser

### XACROParser Class

The `XACROParser` provides native, pure-Python resolution of XACRO files. It handles macros, properties, math expressions, and conditional blocks without external ROS dependencies.

```{eval-rst}
.. autoclass:: linkforge.core.parsers.xacro_parser.XACROParser
   :members:
   :undoc-members:
   :show-inheritance:
```

### XacroResolver Class (Internal)

The internal engine used by `XACROParser` for hierarchical property resolution and macro substitution.

```{eval-rst}
.. autoclass:: linkforge.core.parsers.xacro_parser.XacroResolver
   :members:
   :undoc-members:
```

:::{note}
**Structural Caching (v1.4.0)**: `XacroResolver` implements a two-phase approach for
large modular robot cascades. In the **Structural Phase**, all `xacro:include` tags are
resolved once into an in-memory template tree. In the **Evaluation Phase**, arguments and
conditional blocks are injected into the cached tree. This means a single Xacro file can
be evaluated many times with different parameters (e.g., different `prefix=` values for
two arms) without re-reading or re-parsing any files.
:::

---

## SRDF Parser

The SRDF parser is documented with the rest of the SRDF layer (models, parser, generator)
on the dedicated [SRDF reference page](srdf.md).

## Usage Examples

### Parse XACRO File

To resolve a XACRO file into a plain XML string (format-agnostic):

```python
from linkforge.core.parsers import XACROParser
from pathlib import Path

# Returns a plain XML string
xml_string = XACROParser().resolve(Path("robot.urdf.xacro"))
```

To parse a XACRO file directly into a Robot model (canonical usage):

```python
from linkforge.core.parsers import URDFParser
from pathlib import Path

# Natively resolves XACRO then parses URDF
robot = URDFParser().parse_xacro(Path("robot.urdf.xacro"))
print(f"Loaded robot: {robot.name}")
```

### Parse URDF File

```python
from linkforge.core.parsers import URDFParser
from pathlib import Path

robot = URDFParser().parse(Path("my_robot.urdf"))
print(f"Loaded robot: {robot.name}")
```

### Parse URDF String

```python
from linkforge.core.parsers import URDFParser

urdf_content = """<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link"/>
</robot>"""

robot = URDFParser().parse_string(urdf_content)
```

### Robustness & Security

The parser includes professional-grade protections for production robotics:

*   **Duplicate Name Resolution**: Re-names conflicting link/joint names (e.g., `link_duplicate_1`) to preserve kinematic tree integrity while alerting the user.
*   **DoS Protection**: Enforces a maximum XML depth of 2,000 levels and file size (100 MB) to prevent "XML Bomb" attacks.
*   **O(1) Memory Efficiency**: All core parsers use iterative processing to handle massive robot descriptions with a constant, low memory footprint.
*   **Path Sandboxing**: Validates all mesh paths to prevent directory traversal and ensure assets remain within authorized project folders.
*   **Secured Math Environment**: XACRO expressions are evaluated in a hardened sandbox that prevents access to dangerous Python built-ins or private `__dunder__` methods.
*   **XACRO Debugging Support**: Natively evaluates and routes `xacro.warning()`, `xacro.error()`, `xacro.fatal()`, and `xacro.message()` calls to the LinkForge Python logger.
*   **Resilient Skip**: Malformed geometry or broken joint references are logged as warnings, allowing the rest of the robot to load successfully.
