# ⚖️ Configuring Physics & Inertia

LinkForge provides powerful tools to ensure your robot's physics simulation is stable and accurate.

## Automatic Inertia Calculation

By default, LinkForge calculates the mass properties (Inertia Tensor and Center of Mass) automatically based on the geometry of your link.

1. Select your Link object.
2. Go to the **LinkForge Panel** > **Physics Settings**.
3. Ensure **Auto-Calculate Inertia** is **CHECKED**.
4. Set the **Mass** (kg).

The inertia tensor will be computed using the bounding box (for primitives) or the mesh volume approximation.

:::{important}
For complex meshes, the automatic calculation approximates the shape as a solid primitives (Box/Cylinder/Sphere) based on the visual geometry dimensions. For higher precision, consider decomposing your mesh or providing custom inertia values.
:::

## Manual Center of Mass (Inertial Origin)

Sometimes, the geometric center is not the physical center of mass (e.g., a battery pack inside a chassis). You can manually offset the Center of Mass (COM).

1. **Uncheck** "Auto-Calculate Inertia".
2. You will see new fields for **Inertial Origin**:
   - **Pos**: XYZ offset from the link origin.
   - **Rot**: RPY rotation of the inertia frame.

### 🎯 Visualizing the Center of Mass
When you modify the Inertial Origin:
- An **Orange/White Axis** wireframe appears in the viewport.
- This represents the **Principal Axes of Inertia**.
- Moving the "Pos" values will move this wireframe relative to the link origin.

### Example: Lowering the COM
To make a mobile robot more stable, you often want the center of mass to be low.
1. Uncheck **Auto-Calculate**.
2. Set **Inertial Origin Z** to a negative value (e.g., `-0.05`).
3. You will see the inertia visualization box shift downwards.

## Troubleshooting Physics

### "Exploding" Robot in Gazebo
If your robot explodes or flies away immediately upon spawning:

1. **Check for Negative/Zero Inertia**: Run the LinkForge **Validator**. It will catch zero-mass links or invalid tensors.
2. **Check Collisions**: Ensure adjacent links have collisions that don't overlap in the "Zero Configuration".  Use the `show_in_front` X-Ray view to inspect internal collisions.
3. **Inertia Too Small**: Very small inertia values (like `1e-9`) can cause numerical instability in physics engines. Try increasing the mass or size slightly if possible, or bundle small parts into a larger parent link.
