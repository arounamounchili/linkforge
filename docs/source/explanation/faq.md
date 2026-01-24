# FAQ

Frequently asked questions about LinkForge, its compatibility, and future development.

## Does LinkForge support ROS 1?
While LinkForge exports standard URDF files that are technically compatible with ROS 1, our primary focus is **ROS 2** and **Gazebo Sim (Ignition)**. We use ROS 2 Gazebo plugins and naming conventions by default.

## Can I use exported models in Unreal Engine or Unity?
Yes! Since LinkForge exports standard URDFs, you can use any URDF importer for these engines. However, the specialized Gazebo/ROS 2 control tags will likely be ignored by those engines.

## How do I contribute a new sensor type?
We welcome contributions! Please check our **[Contributing Guide](../CONTRIBUTING.md)** for developer setup instructions. Adding a sensor involves:
1. Defining the sensor model in `linkforge/core/models/sensor.py`.
2. Adding a parser in `linkforge/core/parsers/urdf_parser.py`.
3. Adding a generator in `linkforge/core/urdf_generator.py`.

## Why does LinkForge calculate inertia automatically?
Physics engines are extremely sensitive to incorrect inertia tensors. A common cause of "exploding" simulations is a mass that is too large for its inertia. LinkForge uses proven geometric formulas to ensure your robot stays physically stable.

## What version of Blender is required?
LinkForge is designed for **Blender 4.2+**, utilizing the new Extension Platform system.

## Where did the "Transmissions" panel go?
In v1.2.0, we replaced the manual "Transmission" workflow with a modern **Control Dashboard**. Enable **Use ROS2 Control** in the Control panel to access it. This new system is faster, less error-prone, and fully supports ROS 2 standards.
