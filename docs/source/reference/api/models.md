# Data Models

Core data structures for representing robots.

## Robot

```{eval-rst}
.. autoclass:: linkforge.core.Robot
   :members:
   :undoc-members:
   :show-inheritance:
```

## Link

```{eval-rst}
.. autoclass:: linkforge.core.Link
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.Visual
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.Collision
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.Inertial
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.InertiaTensor
   :members:
   :undoc-members:
   :show-inheritance:
```

.. autoclass:: linkforge.core.models.link.LinkPhysics
   :members:
   :undoc-members:
   :show-inheritance:

## Joint

```{eval-rst}
.. autoclass:: linkforge.core.Joint
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.JointType
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.JointLimits
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.JointDynamics
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.JointMimic
   :members:
   :undoc-members:
   :show-inheritance:
```

.. autoclass:: linkforge.core.models.joint.JointSafetyController
   :members:
   :undoc-members:
   :show-inheritance:

## Geometry

```{eval-rst}
.. autoclass:: linkforge.core.Box
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.Cylinder
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.Sphere
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.Mesh
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.Vector3
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.Transform
   :members:
   :undoc-members:
   :show-inheritance:
```

.. autoclass:: linkforge.core.models.geometry.GeometryType
   :members:
   :undoc-members:
   :show-inheritance:

## Sensor

```{eval-rst}
.. autoclass:: linkforge.core.Sensor
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.SensorType
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.CameraInfo
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.LidarInfo
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.IMUInfo
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.GPSInfo
   :members:
   :undoc-members:
   :show-inheritance:
```

.. autoclass:: linkforge.core.models.sensor.ContactInfo
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.sensor.ForceTorqueInfo
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.sensor.SensorNoise
   :members:
   :undoc-members:
   :show-inheritance:

## Transmission

Standard URDF transmission model for ros_control/ros2_control integration.

While `Ros2Control` provides a modern dashboard-based workflow, `Transmission` remains fully supported for compatibility and standard URDF workflows.

```{eval-rst}
.. autoclass:: linkforge.core.Transmission
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.TransmissionJoint
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.TransmissionActuator
   :members:
   :undoc-members:
   :show-inheritance:
```

## Material

```{eval-rst}
.. autoclass:: linkforge.core.Material
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.Color
   :members:
   :undoc-members:
   :show-inheritance:
```

## Gazebo

```{eval-rst}
.. autoclass:: linkforge.core.GazeboElement
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.GazeboPlugin
   :members:
   :undoc-members:
   :show-inheritance:
```

## ROS2 Control

```{eval-rst}
.. autoclass:: linkforge.core.Ros2Control
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.Ros2ControlJoint
   :members:
   :undoc-members:
.. autoclass:: linkforge.core.models.ros2_control.Ros2ControlSensor
   :members:
   :undoc-members:
   :show-inheritance:
```
