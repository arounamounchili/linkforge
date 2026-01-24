# Reference: Robot Structure & Mapping

This document provides a technical specification of how LinkForge maps Blender objects to URDF elements.

## Object Mapping

| URDF Element | Blender Object Type | Display Type | Notes |
| :--- | :--- | :--- | :--- |
| **Link** | `Empty` | Plain Axes | The coordinate frame of the link. |
| **Visual** | `Mesh` | Textured/Solid | Must be a child of a Link Empty. |
| **Collision** | `Mesh` | Wireframe | Must be a child of a Link Empty. |
| **Joint** | `Empty` | Arrows | Colored axes (RGB for XYZ). |
| **Sensor** | `Empty` | Sphere | Wireframe sphere. Must be a child of a Link Empty. |


## Naming Conventions

LinkForge uses specific suffixes to identify the role of mesh objects within a link.

| Suffix | Role | Export Behavior |
| :--- | :--- | :--- |
| `_visual` | **Visual Geometry** | Exported to the `<visual>` tag. |
| `_collision` | **Collision Geometry** | Exported to the `<collision>` tag. |

### Sanitization Rules
During export, all object names are sanitized to remain compliant with the URDF specification:
1.  **Spaces** are replaced with underscores `_`.
2.  **Special characters** (except underscores and hyphens) are removed.
3.  **Leading numbers** are prefixed with `_` or `link_`.

## Hierarchy Rules

### 1. Link Container
A Link is not just a mesh; it is a container.
- All visual and collision meshes **must** be direct children of the Link Empty.
- A Link Empty **must** have its `is_robot_link` property set to `True`.

### 2. Joint Connectivity
Joints in LinkForge operate differently than standard Blender parent-child relationships:
- A Joint Empty is **not** parented to its child link in Blender.
- Instead, the Joint Empty stores recursive references to the **Parent Link** and **Child Link** in its properties.
- This allows LinkForge to build a kinematic tree that stays 100% compliant with URDF, regardless of how objects are organized in the Blender Outliner.

### 3. Coordinate Systems
Both Blender and URDF use **Z-up, right-handed** coordinate systems. LinkForge uses **direct 1:1 coordinate mapping** (no rotations applied during import/export).

**Coordinate Conventions:**
- **Blender Default:** Z-up, Y-forward, X-right
- **URDF/ROS (REP-103):** Z-up, X-forward, Y-left

**Recommendation:** Model your robot using ROS conventions (X-forward) in Blender for intuitive joint configuration, even though Blender's default is Y-forward.

- **Joint Axes**: Selecting "X", "Y", or "Z" in the Joint panel assigns the exact vector (e.g., `1 0 0`) based on the Joint Empty's local orientation.
- **Origins**: The `location` and `rotation` of Empties in Blender map directly to the `<origin>` tag in URDF (XYZ position, RPY rotation).


## Property Storage
LinkForge stores all metadata as **Custom Properties** on the Blender objects. These can be inspected in the "Custom Properties" panel of the Object Data tab, though it is recommended to use the LinkForge Sidebar for editing.

| Component | Property Prefix |
| :--- | :--- |
| **Link** | `linkforge.*` |
| **Joint** | `linkforge_joint.*` |
| **Transmission** | `linkforge_transmission.*` |
| **Sensor** | `linkforge_sensor.*` |
