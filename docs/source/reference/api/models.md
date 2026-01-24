# Data Models

Core data structures for representing robots.

## Robot

```{eval-rst}
.. autoclass:: linkforge.core.models.robot.Robot
   :members:
   :undoc-members:
   :show-inheritance:
```

## Link

```{eval-rst}
.. autoclass:: linkforge.core.models.link.Link
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.link.Visual
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.link.Collision
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.link.Inertial
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.link.InertiaTensor
   :members:
   :undoc-members:
   :show-inheritance:
```

## Joint

```{eval-rst}
.. autoclass:: linkforge.core.models.joint.Joint
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.joint.JointType
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.joint.JointLimits
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.joint.JointDynamics
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.joint.JointMimic
   :members:
   :undoc-members:
   :show-inheritance:
```

## Geometry

```{eval-rst}
.. autoclass:: linkforge.core.models.geometry.Box
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.geometry.Cylinder
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.geometry.Sphere
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.geometry.Mesh
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.geometry.Vector3
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.geometry.Transform
   :members:
   :undoc-members:
   :show-inheritance:
```

## Sensor

```{eval-rst}
.. autoclass:: linkforge.core.models.sensor.Sensor
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.sensor.SensorType
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.sensor.CameraInfo
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.sensor.LidarInfo
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.sensor.IMUInfo
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.sensor.GPSInfo
   :members:
   :undoc-members:
   :show-inheritance:
```

## Transmission (Legacy)

Legacy support for standard URDF transmissions. Modern workflows use `Ros2Control`.

```{eval-rst}
.. autoclass:: linkforge.core.models.transmission.Transmission
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.transmission.TransmissionJoint
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.transmission.TransmissionActuator
   :members:
   :undoc-members:
   :show-inheritance:
```

## Material

```{eval-rst}
.. autoclass:: linkforge.core.models.material.Material
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.material.Color
   :members:
   :undoc-members:
   :show-inheritance:
```

## Gazebo

```{eval-rst}
.. autoclass:: linkforge.core.models.gazebo.GazeboElement
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.gazebo.GazeboPlugin
   :members:
   :undoc-members:
   :show-inheritance:
```

## ROS2 Control

```{eval-rst}
.. autoclass:: linkforge.core.models.ros2_control.Ros2Control
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: linkforge.core.models.ros2_control.Ros2ControlJoint
   :members:
   :undoc-members:
   :show-inheritance:
```
