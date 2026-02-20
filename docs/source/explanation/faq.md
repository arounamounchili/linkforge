# FAQ

Frequently asked questions about LinkForge, its compatibility, and future development.

## Does LinkForge support ROS 1?
While LinkForge exports standard URDF files that are technically compatible with ROS 1, our primary focus is **ROS 2** and **Gazebo Sim (Ignition)**. We use ROS 2 Gazebo plugins and naming conventions by default.

## Does LinkForge require a ROS installation?
**No.** For core robotic asset management (Import/Export, Mesh Resolution, XACRO Parsing), LinkForge is **completely ROS-agnostic**. It uses a sophisticated "Hybrid Resolver" that can find your packages by searching your directory structure, even on standalone Windows or macOS systems without ROS 2 installed.

## Can I use exported models in Unreal Engine or Unity?
Yes! Since LinkForge exports standard URDFs, you can use any URDF importer for these engines. However, the specialized Gazebo/ROS 2 control tags will likely be ignored by those engines.

## Why does LinkForge calculate inertia automatically?
Physics engines are extremely sensitive to incorrect inertia tensors. A common cause of "exploding" simulations is a mass that is too large for its inertia. As a **Linter for Robotics**, LinkForge uses proven geometric formulas to ensure your robot stays physically stable and catches invalid inertia data before it reaches your simulator.

## What mesh formats should I use? (DAE vs. GLB)
While LinkForge supports **DAE (Collada)** for legacy compatibility (supported in Blender 4.x), we strongly recommend **glTF/GLB** for modern workflows. GLB is the professional standard for PBR materials and is natively supported by Gazebo Sim (Garden+) and modern RViz2 configurations. LinkForge acts as a bridge to help you migrate your assets to these modern formats.

## Why does LinkForge flag "Multiple Root Links"?
This is a "Silent Error" that standard XML parsers often miss. A valid robot must have a single root link (the base). If links like hands or sensors are disconnected in the hierarchy, LinkForge's linter will flag them. This prevents detached physics and "ghost" joints in your simulation.

## How do I contribute?
LinkForge is a community-driven project. We welcome contributions for new sensor types, platform adapters, or bug fixes. Please check our **[Contributing Guide](../CONTRIBUTING.md)** for developer setup and technical standards.

## What version of Blender is required?
LinkForge is designed for **Blender 4.2+**, utilizing the new Extension Platform system.

## Where did the "Transmissions" panel go?
In v1.2.0, we replaced the manual "Transmissions" UI panel with the modern **Control Dashboard**. Enable **Use ROS2 Control** in the Control panel to access it. This new system is faster, less error-prone, and fully supports ROS 2 standards.

**Important:** The `Transmission` data model is still fully supported for import/export. URDFs with `<transmission>` tags will import correctly. However, the Control Dashboard provides a more intuitive workflow for configuring ros2_control hardware interfaces.
