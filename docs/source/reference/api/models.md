# Data Models

Core data structures for representing robots.

## Robot

```{eval-rst}
.. autoclass:: linkforge_core.models.robot.Robot
   :members:
   :undoc-members:
   :show-inheritance:
```

## Link

```{eval-rst}
.. autoclass:: linkforge_core.models.link.Link
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.link.Visual
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.link.Collision
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.link.Inertial
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.link.InertiaTensor
   :members:
   :undoc-members:
   :show-inheritance:
```

## Joint

```{eval-rst}
.. autoclass:: linkforge_core.models.joint.Joint
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.joint.JointType
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.joint.JointLimits
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.joint.JointDynamics
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.joint.JointMimic
   :members:
   :undoc-members:
   :show-inheritance:
```

## Geometry

```{eval-rst}
.. autoclass:: linkforge_core.models.geometry.Box
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.geometry.Cylinder
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.geometry.Sphere
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.geometry.Mesh
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.geometry.Vector3
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.geometry.Transform
   :members:
   :undoc-members:
   :show-inheritance:
```

## Sensor

```{eval-rst}
.. autoclass:: linkforge_core.models.sensor.Sensor
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.sensor.SensorType
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.sensor.CameraInfo
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.sensor.LidarInfo
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.sensor.IMUInfo
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.sensor.GPSInfo
   :members:
   :undoc-members:
   :show-inheritance:
```

## Transmission

Standard URDF transmission model for ros_control/ros2_control integration.

While `Ros2Control` provides a modern dashboard-based workflow, `Transmission` remains fully supported for compatibility and standard URDF workflows.

```{eval-rst}
.. autoclass:: linkforge_core.models.transmission.Transmission
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.transmission.TransmissionJoint
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.transmission.TransmissionActuator
   :members:
   :undoc-members:
   :show-inheritance:
```

## Material

```{eval-rst}
.. autoclass:: linkforge_core.models.material.Material
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.material.Color
   :members:
   :undoc-members:
   :show-inheritance:
```

## Gazebo

```{eval-rst}
.. autoclass:: linkforge_core.models.gazebo.GazeboElement
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.gazebo.GazeboPlugin
   :members:
   :undoc-members:
   :show-inheritance:
```

## ROS2 Control

```{eval-rst}
.. autoclass:: linkforge_core.models.ros2_control.Ros2Control
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge_core.models.ros2_control.Ros2ControlJoint
   :members:
   :undoc-members:
   :show-inheritance:
```
