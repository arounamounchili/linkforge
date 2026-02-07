"""Blender Property Groups for robot links.

These properties are stored on Blender objects and define link characteristics.
"""

from __future__ import annotations

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    StringProperty,
)
from bpy.types import PropertyGroup

from ...linkforge_core.utils.string_utils import sanitize_name as sanitize_urdf_name


def get_link_name(self):
    """Getter for link_name - mirrors and sanitizes the Blender object name."""
    if not self.id_data:
        return ""
    return sanitize_urdf_name(self.id_data.name)


def set_link_name(self, value):
    """Setter for link_name - updates object name and children."""
    if not value or not self.id_data:
        return

    # Sanitize link name for URDF (remove invalid characters)
    sanitized_name = sanitize_urdf_name(value)

    # Update object name to match link name
    # Note: Blender may append .001, .002 etc if name conflicts exist
    if self.id_data.name != sanitized_name:
        self.id_data.name = sanitized_name

    # Update visual and collision children names to match
    for child in self.id_data.children:
        child_lower = child.name.lower()
        if "_visual" in child_lower:
            # Preserve any suffix like _visual_02
            if "_visual_" in child_lower:
                # Extract suffix (e.g., "_02")
                parts = child.name.split("_visual_")
                if len(parts) == 2:
                    new_name = f"{sanitized_name}_visual_{parts[1]}"
                    if child.name != new_name:
                        child.name = new_name
            else:
                new_name = f"{sanitized_name}_visual"
                if child.name != new_name:
                    child.name = new_name
        elif "_collision" in child_lower:
            # Preserve any suffix like _collision_02
            if "_collision_" in child_lower:
                parts = child.name.split("_collision_")
                if len(parts) == 2:
                    new_name = f"{sanitized_name}_collision_{parts[1]}"
                    if child.name != new_name:
                        child.name = new_name
            else:
                new_name = f"{sanitized_name}_collision"
                if child.name != new_name:
                    child.name = new_name


def update_collision_quality(self, context):
    """Update collision mesh preview when quality changes.

    This provides live feedback to the user as they adjust the quality slider,
    showing them exactly how the exported collision mesh will look.
    """
    # Only update if this object is a link with collision
    if not self.is_robot_link:
        return

    # Get the object - try multiple ways since context might vary
    obj = None
    if hasattr(context, "object") and context.object:
        obj = context.object
    elif hasattr(context, "active_object") and context.active_object:
        obj = context.active_object

    if obj is None:
        return

    # IMPORTANT: If the selected object is a collision child, get its parent link
    # This happens after regeneration when Blender selects the new collision object
    if (
        obj.parent
        and hasattr(obj.parent, "linkforge")
        and obj.parent.linkforge.is_robot_link
        and "_collision" in obj.name.lower()
    ):
        obj = obj.parent
    # Check if there's a collision child
    collision_children = [c for c in obj.children if "_collision" in c.name.lower()]
    if not collision_children:
        return

    # IMPORTANT: Skip update if collision was imported from URDF
    # We want to preserve imported custom collision geometry and origins.
    # The 'imported_from_urdf' flag is set in core_to_blender.py during import.
    for col in collision_children:
        if col.get("imported_from_urdf"):
            return

    # Schedule regeneration (debounced via timer to prevent lag)
    from ..operators.link_ops import schedule_collision_preview_update

    schedule_collision_preview_update(obj)


def update_auto_inertia_toggle(self, context):
    """Enable visualization when switching to manual inertia."""
    if not self.use_auto_inertia:
        # User switched to Manual Mode -> Enable visualization
        from ..visualization.inertia_gizmos import ensure_inertia_handler

        ensure_inertia_handler()


class LinkPropertyGroup(PropertyGroup):
    """Properties for a robot link stored on a Blender object."""

    # Link identification
    is_robot_link: BoolProperty(  # type: ignore
        name="Is Robot Link",
        description="Mark this object as a robot link",
        default=False,
    )

    link_name: StringProperty(  # type: ignore
        name="Link Name",
        description="Name of the link in URDF (must be unique)",
        maxlen=64,
        get=get_link_name,
        set=set_link_name,
    )

    # Inertial properties
    use_auto_inertia: BoolProperty(  # type: ignore
        name="Auto-Calculate Inertia",
        description="Let LinkForge calculate physics properties from the 3D shape (recommended)",
        default=True,
        update=update_auto_inertia_toggle,
    )

    mass: FloatProperty(  # type: ignore
        name="Mass",
        description="Weight of this link in kilograms (for physics simulation)",
        default=1.0,
        min=0.0,
        soft_max=1000.0,
        max=1000000.0,
        unit="MASS",
        precision=3,
    )

    # Manual inertia tensor (when auto_inertia is disabled)
    inertia_ixx: FloatProperty(  # type: ignore
        name="Ixx",
        description="Moment of inertia around X-axis - resistance to rotation (kg⋅m²)",
        default=1.0,
        min=0.0,
        precision=6,
    )

    inertia_ixy: FloatProperty(  # type: ignore
        name="Ixy",
        description="Product of inertia XY component - coupling between X and Y rotations (kg⋅m²)",
        default=0.0,
        precision=6,
    )

    inertia_ixz: FloatProperty(  # type: ignore
        name="Ixz",
        description="Product of inertia XZ component - coupling between X and Z rotations (kg⋅m²)",
        default=0.0,
        precision=6,
    )

    inertia_iyy: FloatProperty(  # type: ignore
        name="Iyy",
        description="Moment of inertia around Y-axis - resistance to rotation (kg⋅m²)",
        default=1.0,
        min=0.0,
        precision=6,
    )

    inertia_iyz: FloatProperty(  # type: ignore
        name="Iyz",
        description="Product of inertia YZ component - coupling between Y and Z rotations (kg⋅m²)",
        default=0.0,
        precision=6,
    )

    inertia_izz: FloatProperty(  # type: ignore
        name="Izz",
        description="Moment of inertia around Z-axis - resistance to rotation (kg⋅m²)",
        default=1.0,
        min=0.0,
        precision=6,
    )

    inertia_origin_xyz: FloatVectorProperty(  # type: ignore
        name="Inertia Position",
        description="Position of the center of mass relative to the link frame (meters)",
        default=(0.0, 0.0, 0.0),
        size=3,
        precision=3,
        unit="LENGTH",
    )

    inertia_origin_rpy: FloatVectorProperty(  # type: ignore
        name="Inertia Rotation",
        description="Rotation of the principal axes of inertia relative to the link frame (radians, XYZ order)",
        default=(0.0, 0.0, 0.0),
        size=3,
        precision=3,
        unit="ROTATION",
    )

    collision_type: EnumProperty(  # type: ignore
        name="Collision Type",
        description="Type of collision geometry to generate",
        items=[
            ("AUTO", "Auto", "Automatically detect primitive shape or use convex hull"),
            ("BOX", "Bounding Box", "Axis-aligned bounding box around the mesh"),
            ("SPHERE", "Bounding Sphere", "Spherical bounding volume around the mesh"),
            ("CYLINDER", "Bounding Cylinder", "Cylindrical bounding volume around the mesh"),
            ("CONVEX_HULL", "Convex Hull", "Generate convex hull from mesh"),
        ],
        default="AUTO",
    )

    collision_quality: FloatProperty(  # type: ignore
        name="Collision Quality",
        description=(
            "Mesh detail preserved in collision geometry (100% = full detail, 50% = half the faces). "
            "Lower values = faster physics simulation. Imported collision defaults to 100%"
        ),
        default=50.0,  # 50% for user-created collision (good balance)
        min=1.0,  # At least 1% to avoid empty meshes
        max=100.0,  # 100% = no simplification
        precision=0,  # Show as integer (no decimals)
        subtype="PERCENTAGE",  # Display with % symbol
        update=update_collision_quality,  # Live preview callback
    )

    # Material properties
    use_material: BoolProperty(  # type: ignore
        name="Export Material",
        description="Export color/appearance to URDF (enabled by default for auto-created materials)",
        default=False,  # Set dynamically by operator based on material creation
    )


# Registration
def register():
    """Register property group."""
    try:
        bpy.utils.register_class(LinkPropertyGroup)
    except ValueError:
        # If already registered (e.g. from reload), unregister first to ensure clean state
        bpy.utils.unregister_class(LinkPropertyGroup)
        bpy.utils.register_class(LinkPropertyGroup)

    bpy.types.Object.linkforge = bpy.props.PointerProperty(type=LinkPropertyGroup)


def unregister():
    """Unregister property group."""
    import contextlib

    with contextlib.suppress(AttributeError):
        del bpy.types.Object.linkforge

    with contextlib.suppress(RuntimeError):
        bpy.utils.unregister_class(LinkPropertyGroup)


if __name__ == "__main__":
    register()
