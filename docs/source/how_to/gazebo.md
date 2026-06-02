# Gazebo Integration

To simulate sensors, controllers, and physics perfectly in Gazebo Classic or Gazebo Harmonic (Ignition), you can attach raw Gazebo plugins directly to your links.

## Adding Gazebo Plugins Programmatically

Use the `.physics()` or `.gazebo_params` configuration to inject custom `<gazebo>` tags into the exported URDF:

```python
from linkforge.core import RobotBuilder

builder = RobotBuilder("gazebo_robot")

builder.link("base_link") \
    .physics(
        mu=0.9,
        mu2=0.9,
        kp=1e5,
        kd=1.0,
        self_collide=False,
        gazebo_params={
            "material": "Gazebo/Black",
            "plugin": {
                "@name": "gazebo_ros_control",
                "@filename": "libgazebo_ros_control.so",
                "robotNamespace": "/",
            }
        }
    )
```

The `gazebo_params` dictionary is automatically serialized into XML elements inside the link's `<gazebo>` tag in the final export.
