# Best Practices

To get the most out of LinkForge and ensure your robot simulations are stable and professional, follow these best practices.

## Modeling for Physics

Physics engines like Gazebo or Bullet are highly sensitive to how geometry and mass are defined.

### 1. Scale and Units
- **Always use Meters**: Blender can use various units, but ROS and URDF expect **Meters**. LinkForge automatically handles basic conversions, but modeling at a 1:1 scale in meters is the safest approach.
- **Apply Scale**: Before creating links or joints, ensure all objects have an applied scale of `(1, 1, 1)`. Select objects and press `Ctrl+A > Scale`.

### 2. Mesh Simplification
- **Collision vs. Visual**: Use high-poly meshes for visuals, but **always** use simplified primitive shapes (Box, Cylinder, Sphere) for colliders. LinkForge allows you to set a primitive as the collision shape for a complex visual mesh.
- **Avoid "Thin" Colliders**: Very thin meshes (like a piece of paper) can cause tunneling errors in physics engines. Give your colliders some thickness.

### 3. Orientation
- **X-Forward, Z-Up**: LinkForge uses **direct 1:1 coordinate mapping** (no automatic rotations). Following the ROS standard (X-axis forward, Y-axis left, Z-axis up) when modeling your robot in Blender will make joint configuration intuitive and match ROS expectations.

---

## Modular XACRO Design

If you are building a complex robot (e.g., a quadruped or a high-DOF arm), modularity is key.

### Use Macros for Repetition
Instead of defining 4 identical leg links manually, use LinkForge's **XACRO Export** settings to group identical meshes.

### Separation of Concerns
Consider organizing your robot into:
- `robot.xacro`: The main assembly file.
- `materials.xacro`: Centralized material definitions to ensure consistent colors.
- `constants.xacro`: Store common values like PID gains or sensor offsets.

---

## Performance Tips

### Limit High-Quality Mesh Usage
Too many high-resolution meshes in a simulation will drop the Real-Time Factor (RTF). Use the **Decimate Modifier** in Blender to reduce triangle counts for your visual meshes before export.

### Sensor Update Rates
Don't set sensor update rates higher than necessary. A 100Hz LiDAR might be overkill for a slow-moving mobile robot and will consume unnecessary CPU cycles.
