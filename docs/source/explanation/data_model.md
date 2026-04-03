# Explanation: The LinkForge Data Model

To understand how LinkForge converts a Blender scene into a URDF, it's important to understand the philosophy behind its data model.

## The Semantic Bridge

LinkForge acts as a **Linter & Semantic Bridge**. In Blender, you are manipulating geometry and lights; in a URDF, you are defining a kinematic tree of physical bodies and constraints.

LinkForge decouples these two worlds by using **Empty Objects** as the primary "anchors" for robot components.

### 1. Why use Empties for Links and Joints?
Standard Blender meshes are designed for rendering. They have scale, rotation, and potentially complex modifiers. If we attached URDF data directly to a mesh, changing the visual representation (e.g., swapping a high-poly wheel for a low-poly wheel) would risk losing the physics data.

By using an **Empty** as the "Link Frame":
- **Independence**: You can swap, scale, or move the visual meshes without affecting the Joint origin or the Inertia frame.
- **Precision**: Empties have no geometry by themselves, ensuring they represent a mathematically pure coordinate frame.
- **Visual Clarity**: Joints are represented by arrows, making it easy to see the axis of rotation at a glance.

> [!TIP]
> **Common Misconception**: Using Blender's `Set Origin to 3D Cursor` on a mesh child of a Link Empty only changes the visual offset of that mesh. It **does not** update the Joint origin. To change where a Link rotates or pivots, you must manipulate the **Joint Empty** or **Link Empty** themselves.


### 2. Composition over Parenting
In a standard Blender hierarchy, you might parent a wheel mesh to a car body mesh. In LinkForge, we use a more structured approach:
- **Link (Empty)**: The root container for a physical body.
- **Visual/Collision (Meshes)**: Children of the Link Empty.
- **Joint (Empty)**: A floating coordinate frame that defines how one Link connects to another.

This structure allows for **non-destructive editing**. You can hide all collision meshes to work on aesthetics, or hide visuals to inspect the physics layout, all without breaking the kinematic tree.

### 3. Automatic Physics & Linting
LinkForge bridges the gap between Blender's visual bounding boxes and URDF's inertia tensors. As a linter, it ensures the connection is physically consistent, automatically calculating a valid inertia tensor based on the link's geometry and total mass.

Note: LinkForge calculates the inertia tensor based on the primary collision or visual geometry and scales it to match the total mass you define in the Link properties.

---

### 4. The Control Layer
Modern robotics (ROS 2) separates the *description* of the robot from its *control*. LinkForge aligns with this by abstracting control data:
- **Interfaces, Not Transmissions**: Instead of manually building transmission chains, you simply declare "I want to control velocity on this joint."
- **Centralized Dashboard**: All control logic is aggregated in the **Control Dashboard**, allowing LinkForge to generate the complex `ros2_control` XML blocks automatically while keeping your viewports clean.

---

:::{tip}
For a technical breakdown of naming conventions and object types, see the [Robot Structure Reference](../reference/robot_structure.md).
:::

---

### 5. The Assembly Layer

While the Blender UI covers the most common workflow, LinkForge also provides a
programmatic assembly API for engineers who need to build robots dynamically in Python.

The `RobotAssembly` class wraps two objects into one coordinated interface:

- **`Robot`** — the kinematic description (links, joints, sensors, physics)
- **`SemanticRobotDescription`** — the semantic overlay (planning groups, named
  poses, collision filters for MoveIt)

This separation mirrors the real-world split between a URDF file (what the robot
looks like) and an SRDF file (how the planner should think about it).

Two assembly patterns are supported:

**Macro-Assembly**: Merging complete pre-built robots together at a named flange.
This is how you would attach a gripper to an arm, or build a factory cell with
multiple robot arms sharing a common world frame.

**Micro-Construction**: Building links and joints one at a time using the fluent
`LinkBuilder` API. This is useful for procedurally generated robots or when you
need precise, programmatic control over every joint parameter.

```python
from linkforge_core.composer.robot_assembly import RobotAssembly

# Both patterns produce the same output: a validated Robot + SRDF ready for
# export to URDF, SRDF, or any future format (MJCF, SDF).
assembly = RobotAssembly("my_robot", ...)
urdf_str = assembly.export_urdf()
srdf_str = assembly.export_srdf()
```

:::{tip}
See the [Composer API reference](../reference/api/composer) for the full API documentation.
:::
