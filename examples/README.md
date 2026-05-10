# LinkForge Examples

This directory contains example URDF and Xacro files to demonstrate the capabilities of LinkForge and provide templates for your own robot models.

## Directory Structure

- **urdf/**: Standalone URDF XML models of various robots (mobile bases, quadrupeds).
- **xacro/**: Modular models using Xacro macros and property evaluation.
  - `diff_drive_robot.xacro`: Main entry point for a modular mobile robot.
  - `inertials.xacro`: Reusable kinematic and inertial macros.
  - `materials.xacro`: Shared materials and global properties.

## Usage

You can import these files directly into Blender using the LinkForge addon, or use them with the LinkForge Python API:

```python
from linkforge_core.parsers.urdf_parser import URDFParser

parser = URDFParser()
robot = parser.parse("examples/urdf/diff_drive_robot.urdf")
print(robot.summary())
```

### Xacro Support

LinkForge also supports Xacro files by automatically evaluating macros and properties during parsing:

```python
from pathlib import Path
from linkforge_core.parsers.urdf_parser import URDFParser

parser = URDFParser()
# Resolve Xacro then parse into a Robot model
robot = parser.parse_xacro(Path("examples/xacro/diff_drive_robot.xacro"))
print(robot.summary())
```
