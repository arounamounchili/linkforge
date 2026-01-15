# How-to: Configure Transmissions & Control

This guide explains how to set up transmissions and `ros2_control` interfaces for your actuated joints.

## 1. Enable Control
To export valid ROS 2 control tags, you must enable the feature. This can be done at any time before export:
1. Go to the **Validate & Export** panel in the LinkForge sidebar.
2. Check **Generate ROS2 Control**.
3. **Hardware System**: This is automatically set to `GazeboSimSystem` (standard for Gazebo Sim). You usually don't need to change this unless you are writing a custom hardware interface.

## 2. Add a Transmission
1. Select the **Joint** you want to actuate (either in the viewport or via the Outliner).
2. Go to the **Control** panel in the LinkForge sidebar.
3. Click **Add Transmission**.

## 3. Configure Hardware Interfaces
A transmission can support multiple hardware interfaces. For most ROS 2 controllers, you should select at least one:
- **Position**: For standard servo-like control.
- **Velocity**: For wheels or continuous rotation.
- **Effort**: For torque-based control.

## 4. Mechanical Reduction
You can set a **Mechanical Reduction** factor (default is 1.0). This generates the `<mechanicalReduction>` tag in the URDF, which is used by some physics engines to simulate gearboxes.
