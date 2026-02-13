# Troubleshooting

This guide addresses common challenges in the LinkForge-to-Simulation workflow, focusing on physics stability, kinematic integrity, and asset management.

## ⚖️ Physics Stability

### Robot "folds," "explodes," or jitters in simulation
**Cause**: Numerical instability, usually caused by inconsistent mass data, misaligned origins, or "Double-Offsets" from world-baked meshes.
**Solutions**:
- **Check Inertial Values**: Tiny or zero values (`ixx`, `iyy`, `izz`) cause solvers to fail. Ensure **Auto-Calculate Inertia** is enabled in the **Links** panel.
- **Mesh Centering**: LinkForge automatically localizes your meshes during export (Geometric Centering). This ensures the STL folder contains 0,0,0-centered meshes, while the URDF handles the offset. This prevents "double-transformation" explosions.
- **Apply Scale and Rotation**: Before marking a mesh as a link, ensure its scale is `1.0`. While LinkForge fixes scaling during collision generation, keeping your visual objects at `1.0` scale in Blender prevents mathematical edge cases. `Ctrl+A > All Transforms` is the best practice.
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
**Cause**: The joint's local axis orientation is reversed or the Euler order is mismatched.
**Solutions**:
- **Rotation Order**: LinkForge uses a strict **XYZ extrinsic RPY** mapping. Ensure your Blender objects are set to `XYZ` rotation mode. LinkForge enforces this automatically on Link and Joint creation.
- **Visual Check**: Follow the RGB arrows at the joint origin.
- **Change the Axis**: In the **Joints** panel, select the appropriate **Axis** button (X, Y, Z, or CUSTOM).

### Values "varying" by tiny decimals (1e-6)
**Cause**: Floating-point drift and "Quantization." This occurs when converting between Matrices (Blender internal) and RPY (URDF).
**Solution**:
- This is normal and expected in robotics.
- To "lock" a value, manually type the desired number (e.g., `0` or `1.5708`) into the LinkForge property field. This prevents the tiny matrix flickers from affecting your export over multiple design cycles.

## 📦 Asset & Export Management

### Invisible Robot or Broken Meshes in Gazebo
**Cause**: Incorrect mesh export paths or missing package metadata.
**Solutions**:
- **Check Package Name**: LinkForge uses a **Hybrid Path Resolver**. It automatically searches your directory tree to find the package root for `package://` and `$(find ...)` paths. Ensure your robot files are stored within a folder structure that includes a `package.xml` or matches the package name.
- **Standalone Support**: Since v1.2.3, you **no longer need a ROS installation** on Windows or macOS for asset resolution; LinkForge performs its own heuristic search to find your meshes.

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

## 🛡️ Context & Mode Reliability

### "Context is Incorrect" or Operator failed
**Status**: Resolved
**Explanation**: LinkForge uses a robust **Context Guard** and **Mode Switcher**.
- **Edit Mode Support**: You can run any LinkForge command (like generating collisions or adding sensors) while in **Edit Mode**. LinkForge will safely switch to Object Mode internally and then automatically **restore your Edit Mode** so your flow is never interrupted.
- **Background Execution**: Background processes like imports or timers now use explicit context overrides, preventing the common "Context is Incorrect" crashes that plague many standard Blender add-ons.

## 💡 Pro Tips for Experts

- **Selection via Outliner**: When your robot gets complex, clicking small joint empties in the 3D Viewport can be difficult. Use the **Blender Outliner** to select components by name; the LinkForge panels will update instantly.
- **Naming Discipline**: While **LinkForge automatically sanitizes** names during export (e.g., "My Arm!" -> `my_arm`), it is best practice to use alphanumeric characters and underscores in Blender. This ensures your Blender Outliner matches your generated ROS 2 topics 1:1.
- **Apply Location**: Besides Scale and Rotation, ensure your "Root Link" (usually `base_link`) is at `(0, 0, 0)` in Blender's world space before starting your build. While LinkForge **automatically centers child mesh data**, keep your robot centered at the world origin for clean simulation spawning.
- **Save Often**: LinkForge stores its data inside the `.blend` file. Save your work frequently to preserve your kinematic tree and sensor configurations.
