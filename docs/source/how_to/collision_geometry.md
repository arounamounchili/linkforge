# How-to: Manage Collision Geometry

Collision geometry defines the physical shape of your robot used by physics engines (like ODE or Bullet) to calculate contacts, friction, and collisions.

## 1. Why Collision Matters
While your **Visual** meshes can be high-poly and complex for aesthetics, **Collision** meshes should be as simple as possible (primitives or simplified meshes) to ensure simulation performance and stability.

### The "Source of Truth"
LinkForge follows a **"What You See Is What You Export"** philosophy:
*   **Object Mode State**: The exported visual and collision geometry is always based on the current state of your meshes in Blender's Object Mode.
*   **Edit Mode Support**: You can make complex modifications in Edit Mode (moving vertices, deleting faces). LinkForge analyzes the underlying mesh data at the moment of export or collision generation. There is no need to manually "bake" or "apply" every minor mesh change; LinkForge reads the current mesh buffer directly.

## 2. Generating Collisions
LinkForge makes it easy to generate optimized collision geometry from your visuals:

1. Select a **Link** (the Empty object) or any of its children.
2. Go to the **Links** panel in the LinkForge sidebar.
3. Click **Generate Collision**.

### Collision Types
In the settings below the button, you can choose the generation strategy:
- **Auto-Detect**: LinkForge analyzes the visual mesh. If it resembles a cube, sphere, or cylinder, it creates a corresponding primitive. Otherwise, it exports the mesh (with optional simplification).
- **Box/Sphere/Cylinder**: Force LinkForge to use a specific primitive shape based on the visual's bounding box.
- **Simplified Mesh**: Decimates the visual mesh to a target poly-count. Best for complex, non-primitive parts.

:::{tip}
**Wheels and Discs**: Always use **Cylinder** for wheels. While a squashed **Sphere** might look right in Blender, the URDF standard only supports uniform spheres (one radius). Exporting a flat wheel as a sphere will result in a giant ball in simulation.
:::

## 3. Compound Collisions
If a Link has multiple visual meshes (e.g., a chassis made of several separate parts), LinkForge follows a **"Merged Hull"** strategy:

1. LinkForge identifies all visual children of the Link.
2. It merges these meshes into a single, unified geometry baked in the link's coordinate frame.
3. It performs a **Decimation** operation on this unified mesh to create a single, efficient collision part.

This approach ensures maximum simulation stability, as a single unified collision mesh is far less prone to "snagging" or numerical jitter than a collection of separate overlapping meshes.

## 4. Live Preview & Quality
- **Collision Quality**: Use the slider to decimate the collision mesh. Higher values (1.0) preserve more detail, while lower values simplify the geometry for faster simulation.
- **Toggling Visibility**: Click the **Show/Hide Collision** button in the **Links** panel to inspect the generated geometry. You can also use the wireframe icon in the **Validate & Export** panel to see all robot collisions at once.

## 5. Manual Collisions
If you want to provide your own hand-optimized collision mesh:
1. Create or select a mesh to use as your collision.
2. **Parent the mesh** to your Link object (it must appear as a child in the Blender Outliner).
3. Ensure the mesh name ends with the `_collision` suffix (e.g., `chassis_collision`).
4. LinkForge will automatically detect this and use it instead of generating one.

## 6. Imported Robots & Protected Geometry
When you import a robot from a URDF or XACRO file, LinkForge enters a **Data Integrity Mode** to protect your calibrated physics:

- **Geometry Preservation**: Imported collision meshes (primitives, or raw meshes) are protected from accidental regeneration. This ensures that custom-designed collision shapes are never overwritten.
- **Visual Status Label**: In the **Links** panel, a 🔒 **Imported collision: Geometry preserved** status appears.
- **Quality Slider Lock**: The **Collision Quality** slider is disabled for imported meshes. This prevents accidental mesh degradation of your "Source of Truth" assets.
- **Inertia Protection**: LinkForge also disables **Auto-Calculate Inertia** for imported links, preserving the original `<inertial>` tags from your URDF.

### Overriding Protection
If you decide to discard the imported collision and generate a new one:
1. Click **Regenerate Collision**.
2. This clears the "Imported" flag and creates a new simplified mesh based on current visuals.
3. The **Auto-Calculate Inertia** and **Collision Quality** tools will then be available for this link.

## 7. Understanding Triangulation
When you export a robot and then import it back into Blender, you may notice that the collision meshes look more "triangulated" (showing messier internal edges) than your original mesh.

*   **Technical Reason**: Robot simulators and URDF standards require collision geometry to be stored in the **STL format**. STLs only support **Triangles**.
*   **The Process**: During export, LinkForge automatically triangulates your geometry to meet this standard.
*   **Simulation Fidelity**: This has **zero impact** on physics performance. A flat quad made of two triangles is mathematically identical to a single quad in a physics solver.
