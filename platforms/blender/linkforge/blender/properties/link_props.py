"""Blender Property Groups for robot links.

These properties are stored on Blender objects and define link characteristics.
"""

from __future__ import annotations

import typing

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import Context, PropertyGroup

from ...linkforge_core.utils.string_utils import sanitize_name as sanitize_urdf_name


def get_link_name(self: LinkPropertyGroup) -> str:
    """Getter for link_name - returns the persistent URDF identity."""
    # Prioritize the stored identity to avoid Blender's .001 suffixing
    if self.urdf_name_stored:
        return str(self.urdf_name_stored)

    if not self.id_data:
        return ""
    return sanitize_urdf_name(str(self.id_data.name))


def set_link_name(self: LinkPropertyGroup, value: str) -> None:
    """Setter for link_name - updates persistent identity and object name."""
    if not value or not self.id_data:
        return

    # Sanitize link name for URDF (remove invalid characters)
    sanitized_name = sanitize_urdf_name(value)

    # Store the persistent identity
    self.urdf_name_stored = sanitized_name

    # Update object name to match link name
    # Blender will handle collisions by appending suffixes, but our stored name persists
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


def on_collision_quality_update(self: bpy.types.PropertyGroup, context: Context) -> None:
    """Update collision mesh preview when quality changes.

    This provides live feedback to the user as they adjust the quality slider,
    showing them exactly how the exported collision mesh will look.
    """
    if not self.id_data:
        return

    obj = typing.cast(bpy.types.Object, self.id_data)
    lf = getattr(obj, "linkforge")
    if not lf.is_robot_link:
        return

    # Find collision object
    collision_obj = next((c for c in obj.children if "_collision" in c.name.lower()), None)
    if collision_obj is None:
        return

    # Skip regeneration for imported URDF models to preserve external data
    if "imported_from_urdf" in collision_obj:  # type: ignore[operator]
        return

    # Update ratio in realtime
    from ..operators.link_ops import update_collision_quality_realtime

    update_collision_quality_realtime(obj, collision_obj)


def update_auto_inertia_toggle(self: PropertyGroup, context: Context) -> None:
    """Enable visualization when switching to manual inertia."""
    if not hasattr(self, "use_auto_inertia"):
        return
    if not getattr(self, "use_auto_inertia", True):
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

    # Persistent URDF Identity
    # Decouples logical URDF naming from physical Blender object names (resilient to .001 suffixes)
    urdf_name_stored: StringProperty(  # type: ignore
        name="URDF Name",
        description="Persistent URDF name. Prevents mapping breakage if Blender renames the object",
        default="",
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
            ("AUTO", "Auto", "Automatically detect primitive shape or export as mesh"),
            ("BOX", "Bounding Box", "Axis-aligned bounding box around the mesh"),
            ("SPHERE", "Bounding Sphere", "Spherical bounding volume around the mesh"),
            ("CYLINDER", "Bounding Cylinder", "Cylindrical bounding volume around the mesh"),
            ("MESH", "Mesh (Simplified)", "Generate simplified mesh from visual geometry"),
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
        update=on_collision_quality_update,  # Live preview callback
    )

    # Material properties
    use_material: BoolProperty(  # type: ignore
        name="Export Material",
        description="Export color/appearance to URDF (enabled by default for auto-created materials)",
        default=False,  # Set dynamically by operator based on material creation
    )


# Registration
__all__ = [
    "LinkPropertyGroup",
    "register",
    "unregister",
    "sanitize_urdf_name",
]


def register() -> None:
    """Register property group."""
    try:
        bpy.utils.register_class(LinkPropertyGroup)
    except ValueError:
        # If already registered (e.g. from reload), unregister first to ensure clean state
        bpy.utils.unregister_class(LinkPropertyGroup)
        bpy.utils.register_class(LinkPropertyGroup)

    bpy.types.Object.linkforge = PointerProperty(type=LinkPropertyGroup)  # type: ignore


def unregister() -> None:
    """Unregister property group."""
    import contextlib

    with contextlib.suppress(AttributeError):
        del bpy.types.Object.linkforge  # type: ignore

    with contextlib.suppress(RuntimeError):
        bpy.utils.unregister_class(LinkPropertyGroup)


if __name__ == "__main__":
    register()
