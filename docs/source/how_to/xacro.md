# Xacro Macros

LinkForge provides out-of-the-box support for exporting robots with modular Xacro (XML Macros) capabilities.

## Xacro Support

Xacro allows you to build modular and parameterizable URDF files. LinkForge's export pipeline natively resolves Xacro properties and mathematical expressions.

### Programmatic Usage

You can define Xacro properties and use them in your mathematical expressions when building a robot programmatically:

```python
from linkforge.core import RobotBuilder, box

builder = RobotBuilder("xacro_robot")

# Define a global property
builder.robot.properties["chassis_mass"] = "5.0"
builder.robot.properties["chassis_length"] = "0.4"

# Use expressions in physics or dimensions
builder.link("chassis") \
    .visual(box("${chassis_length}", 0.3, 0.1)) \
    .mass("${chassis_mass}") \
    .root()
```

When you call `.export_urdf()`, the file will be generated as a compliant `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">` and any un-evaluated expressions will be preserved for the `xacro` command-line tool.
