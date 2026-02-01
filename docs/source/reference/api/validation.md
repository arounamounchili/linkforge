# Validation

Robot validation and security checks.

## Robot Validator

```{eval-rst}
.. autoclass:: linkforge_core.validation.validator.RobotValidator
   :members:
   :undoc-members:
   :show-inheritance:
```

## Validation Result

```{eval-rst}
.. autoclass:: linkforge_core.validation.result.ValidationResult
   :members:
   :undoc-members:
   :show-inheritance:
```

## Security

```{eval-rst}
.. automodule:: linkforge_core.validation.security
   :members:
   :undoc-members:
   :show-inheritance:
```

## Usage Examples

### Validate Robot

```python
from linkforge_core.validation.validator import RobotValidator

validator = RobotValidator()
result = validator.validate(robot)

if result.is_valid:
    print("✓ Robot is valid!")
else:
    print("✗ Validation errors:")
    for error in result.errors:
        print(f"  - {error}")

    print("\nWarnings:")
    for warning in result.warnings:
        print(f"  - {warning}")
```

### Security Checks

```python
from linkforge_core.validation.security import validate_mesh_path, find_sandbox_root
from pathlib import Path

# Validate mesh path (prevents path traversal)
try:
    validate_mesh_path(Path("../../etc/passwd"), Path("/tmp"))  # Raises ValueError
except ValueError as e:
    print(f"Security error: {e}")

# Valid paths within sandbox
urdf_dir = Path("/my_robot/urdf")
validate_mesh_path(Path("meshes/robot.stl"), urdf_dir)  # OK

# Auto-detect sandbox root for sibling folder access
urdf_file = Path("/my_robot/urdf/robot.urdf")
sandbox = find_sandbox_root(urdf_file)  # Returns /my_robot
validate_mesh_path(Path("../meshes/part.stl"), urdf_dir, sandbox_root=sandbox)  # OK
```

## Validation Checks

The validator performs the following checks:

### Structure Validation
- Robot has a name
- At least one link exists
- All links have unique names
- All joints have unique names
- Tree structure is valid (no cycles, single root)

### Link Validation
- Links have inertial properties (mass > 0)
- Inertia tensors are physically valid
- Visual and collision geometries are valid

### Joint Validation
- Parent and child links exist
- Joint limits are valid (lower ≤ upper)
- Axis is non-zero for revolute/prismatic joints
- Mimic joints reference existing joints

### Sensor Validation
- Sensors are attached to existing links
- Sensor-specific info is provided
- Update rates are positive

### Security Validation
- Mesh paths don't escape the sandbox root
- Sandbox root auto-detection for sibling folders
- Numeric values are within safe ranges
- XML depth is limited (prevents XML bombs)
