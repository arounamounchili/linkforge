# Validation Checks

The validation system is built from modular, single-responsibility checks.
Each check targets one specific aspect of the robot model and can be run
independently or composed into a full validation pipeline.

## Abstract Base

```{eval-rst}
.. autoclass:: linkforge_core.validation.checks.ValidationCheck
   :members:
   :show-inheritance:
```

## Built-in Checks

```{eval-rst}
.. autoclass:: linkforge_core.validation.checks.HasLinksCheck
   :members:
   :show-inheritance:

.. autoclass:: linkforge_core.validation.checks.DuplicateNameCheck
   :members:
   :show-inheritance:

.. autoclass:: linkforge_core.validation.checks.JointReferenceCheck
   :members:
   :show-inheritance:

.. autoclass:: linkforge_core.validation.checks.TreeStructureCheck
   :members:
   :show-inheritance:

.. autoclass:: linkforge_core.validation.checks.MassPropertiesCheck
   :members:
   :show-inheritance:

.. autoclass:: linkforge_core.validation.checks.GeometryCheck
   :members:
   :show-inheritance:

.. autoclass:: linkforge_core.validation.checks.Ros2ControlCheck
   :members:
   :show-inheritance:

.. autoclass:: linkforge_core.validation.checks.MimicChainCheck
   :members:
   :show-inheritance:
```

---

## Usage Examples

### Run the full validator

The standard workflow — runs all registered checks in order:

```python
from linkforge_core.validation.validator import RobotValidator

validator = RobotValidator()
result = validator.validate(robot)

if not result.is_valid:
    for error in result.errors:
        print(f"[ERROR] {error.title}: {error.message}")
```

### Run a single check in isolation

Useful for targeted testing or when building a custom validation pipeline:

```python
from linkforge_core.validation.checks import TreeStructureCheck
from linkforge_core.validation.result import ValidationResult

result = ValidationResult()
TreeStructureCheck().run(robot, result)

if result.errors:
    print("Tree structure is invalid:")
    for err in result.errors:
        print(f"  - {err.message}")
else:
    print("Tree structure is valid.")
```

### Build a custom validation pipeline

Pick and compose only the checks you need:

```python
from linkforge_core.validation.checks import (
    HasLinksCheck,
    DuplicateNameCheck,
    MassPropertiesCheck,
)
from linkforge_core.validation.result import ValidationResult

checks = [HasLinksCheck(), DuplicateNameCheck(), MassPropertiesCheck()]
result = ValidationResult()

for check in checks:
    check.run(robot, result)

print(f"Errors: {len(result.errors)}, Warnings: {len(result.warnings)}")
```

:::{tip}
All checks are stateless. The same check instance can be run against multiple
robots in sequence without any side effects.
:::
