# Blender Integration

Blender-specific modules for UI and scene integration.

> **Note**: These modules require Blender to run and are not available in standalone Python environments.

## Operators

User actions and commands.

```{eval-rst}
.. automodule:: linkforge.blender.operators.import_ops
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.operators.export_ops
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.operators.link_ops
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.operators.joint_ops
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.operators.sensor_ops
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.operators.control_ops
   :members:
   :undoc-members:
   :show-inheritance:
```

## Properties

Blender scene properties for storing robot data.

```{eval-rst}
.. automodule:: linkforge.blender.properties.robot_props
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.properties.link_props
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.properties.joint_props
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.properties.sensor_props
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.properties.control_props
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.properties.validation_props
   :members:
   :undoc-members:
   :show-inheritance:
```

## Adapters

Conversion between Blender and core models.

```{eval-rst}
.. automodule:: linkforge.blender.converters
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.scene_builder
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.mesh_export
   :members:
   :undoc-members:
   :show-inheritance:
```

## Utilities

Blender-specific helpers.

```{eval-rst}
.. automodule:: linkforge.blender.utils.property_helpers
   :members:
   :undoc-members:
   :show-inheritance:

.. automodule:: linkforge.blender.utils.transform_utils
   :members:
   :undoc-members:
   :show-inheritance:
```

## Usage in Blender

### Accessing from Blender Python Console

```python
import bpy
from linkforge.blender.converters import scene_to_robot

# Convert current scene to robot model
robot = scene_to_robot(bpy.context)

# Access robot data
print(f"Robot: {robot.name}")
for link in robot.links:
    print(f"  Link: {link.name}")
```

### Creating Custom Operators

```python
import bpy
from linkforge.blender.operators import LinkForgeOperator

class LINKFORGE_OT_my_custom_op(LinkForgeOperator):
    bl_idname = "linkforge.my_custom_op"
    bl_label = "My Custom Operation"

    def execute(self, context):
        # Your code here
        self.report({'INFO'}, "Operation complete!")
        return {'FINISHED'}
```
