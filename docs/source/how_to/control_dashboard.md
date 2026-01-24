# How-to: Configure Centralized Control

This guide explains how to set up `ros2_control` interfaces using the centralized **Control Dashboard**.

## 1. Enable Control
To generate valid ROS 2 control tags, enablement is required:
1. Open the **Control** panel in the LinkForge sidebar.
2. Check **Use ROS2 Control**.

This activates the dashboard and sets the default hardware plugin to `GazeboSimSystem`.

## 2. The Control Dashboard
Instead of configuring individual transmissions, LinkForge now provides a unified list of all controlled joints.

### Adding a Joint
1. Click the **Add (+)** button next to the "Joint Interfaces" list.
2. Select a joint from the dropdown menu (only unconfigured joints are shown).

### Configuring Interfaces
Select a joint in the list to reveal its settings:

- **Command Interfaces**: What you *send* to the joint (Position, Velocity, Effort).
- **State Interfaces**: What you *read* from the joint (Position, Velocity, Effort).

:::{tip}
Standard ROS 2 controllers (like `diff_drive_controller`) typically require **Velocity** command interfaces for wheels.
:::

## 3. Global Hardware Settings
In the **Hardware System** section, you can configure:
- **Name**: The name of the `ros2_control` system (default: `GazeboSimSystem`).
- **Plugin**: The C++ hardware plugin to load (default: `gz_ros2_control/GazeboSimSystem`).
- **Gazebo Integration**: Parameters for the Gazebo Classic/Sim plugin definition in URDF.
