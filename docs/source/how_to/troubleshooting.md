# Troubleshooting

This guide addresses common challenges in the LinkForge-to-Simulation workflow, focusing on physics stability, kinematic integrity, and asset management.

## ⚖️ Physics Stability

### Robot "folds," "explodes," or jitters in simulation
**Cause**: Numerical instability, usually caused by inconsistent mass data or misaligned origins.
**Solutions**:
- **Check Inertial Values**: Tiny or zero values (`ixx`, `iyy`, `izz`) cause solvers to fail. Ensure **Auto-Calculate Inertia** is enabled in the **Links** panel.
- **Apply Scale and Rotation**: In Blender, non-uniform scale or un-applied rotations on your visual meshes can lead to incorrect bounding box calculations. Select your meshes and press `Ctrl+A > All Transforms`.
- **Simplify Collision**: High-poly collision meshes cause physics "jitter." Use the **Collision Quality** slider in the **Links** panel to decimate your hulls.

### Joints are "weak" or non-responsive
**Cause**: Mismatch between control expectations and URDF definitions.
**Solutions**:
- **Check Control Dashboard**: Ensure the joint is added to the **Control** panel's dashboard and has a valid Command Interface.
- **Interface Mismatch**: Ensure your hardware interface (Position, Velocity, or Effort) matches your ROS 2 controller configuration.

## 🌳 Kinematic Integrity

### Disconnected Links or "Falling Parts"
**Cause**: Broken chain in the kinematic tree.
**Solutions**:
- **Validation Check**: Run **Validate Robot** in the **Validate & Export** panel. It precisely identifies "island" links not connected to the root.
- **Joint Parenting**: In LinkForge, joints define the parent/child relationship. Ensure every link (except the root) is assigned as a **Child Link** in exactly one joint.

### "Inverted" Rotation or Translation
**Cause**: The joint's local axis orientation is reversed.
**Solutions**:
- **Visual Check**: Follow the RGB arrows at the joint origin.
- **Change the Axis**: In the **Joints** panel, select the appropriate **Axis** button (X, Y, Z, or CUSTOM).

## 📦 Asset & Export Management

### Invisible Robot or Broken Meshes in Gazebo
**Cause**: Incorrect mesh export paths or missing package metadata.
**Solutions**:
- **Check Package Name**: LinkForge uses `package://[robot_name]/meshes/` paths by default. Ensure your `package.xml` matches your **Robot Name** in the **Validate & Export** panel.
- **Global Path Sync**: Verify the **Mesh Directory** name matches your actual folder on disk.

### Properties changing across "Duplicate" Links
**Cause**: Blender's Linked Data (instancing).
**Solutions**:
- If you change the mass of one link and others change unexpectedly, the objects are sharing Mesh Data.
- Fix: `Object > Relations > Make Single User > Object & Data`.

## 🛠️ UI & Viewport

### Viewport is cluttered with giant icons
**Solutions**:
- **Global Scale**: Adjust the **Empty Display > Size** slider in **Blender Preferences > Add-ons > LinkForge**.
- **Hide Helpers**: Toggle the **Show Collisions** checkbox in the **Validate & Export** panel or use the standard Blender **Hide Extras** viewport overlay to see only the robot geometry.

## 💡 Pro Tips for Experts

- **Selection via Outliner**: When your robot gets complex, clicking small joint empties in the 3D Viewport can be difficult. Use the **Blender Outliner** to select components by name; the LinkForge panels will update instantly.
- **Naming Discipline**: While **LinkForge automatically sanitizes** names during export (e.g., "My Arm!" -> `my_arm`), it is best practice to use alphanumeric characters and underscores in Blender. This ensures your Blender Outliner matches your generated ROS 2 topics 1:1.
- **Apply Location**: Besides Scale and Rotation, ensure your "Root Link" (usually `base_link`) is at `(0, 0, 0)` in Blender's world space before starting your build. This ensures the robot spawns correctly at the world origin in simulation.
- **Save Often**: LinkForge stores its data inside the `.blend` file. Save your work frequently to preserve your kinematic tree and sensor configurations.
