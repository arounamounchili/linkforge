# Explanation: The LinkForge Data Model

To understand how LinkForge converts a Blender scene into a URDF, it's important to understand the philosophy behind its data model.

## The Semantic Bridge

LinkForge acts as a **semantic bridge**. In Blender, you are manipulating geometry and lights; in a URDF, you are defining a kinematic tree of physical bodies and constraints.

LinkForge decouples these two worlds by using **Empty Objects** as the primary "anchors" for robot components.

### 1. Why use Empties for Links and Joints?
Standard Blender meshes are designed for rendering. They have scale, rotation, and potentially complex modifiers. If we attached URDF data directly to a mesh, changing the visual representation (e.g., swapping a high-poly wheel for a low-poly wheel) would risk losing the physics data.

By using an **Empty** as the "Link Frame":
- **Independence**: You can swap, scale, or move the visual meshes without affecting the Joint origin or the Inertia frame.
- **Precision**: Empties have no geometry by themselves, ensuring they represent a mathematically pure coordinate frame.
- **Visual Clarity**: Joints are represented by arrows, making it easy to see the axis of rotation at a glance.

### 2. Composition over Parenting
In a standard Blender hierarchy, you might parent a wheel mesh to a car body mesh. In LinkForge, we use a more structured approach:
- **Link (Empty)**: The root container for a physical body.
- **Visual/Collision (Meshes)**: Children of the Link Empty.
- **Joint (Empty)**: A floating coordinate frame that defines how one Link connects to another.

This structure allows for **non-destructive editing**. You can hide all collision meshes to work on aesthetics, or hide visuals to inspect the physics layout, all without breaking the kinematic tree.

### 3. Automatic Physics
LinkForge bridges the gap between Blender's visual bounding boxes and URDF's inertia tensors. Because it understands the Link Frame separately from the meshes, it can automatically calculate a physically consistent inertia tensor based on the link's geometry and total mass.

Note: LinkForge calculates the inertia tensor based on the primary collision or visual geometry and scales it to match the total mass you define in the Link properties.

---

:::{tip}
For a technical breakdown of naming conventions and object types, see the [Robot Structure Reference](../reference/robot_structure.md).
:::
