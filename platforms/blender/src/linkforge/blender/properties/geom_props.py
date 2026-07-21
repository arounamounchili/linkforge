"""Per-mesh geometry properties for visual and collision objects."""

from __future__ import annotations

import typing

import bpy
from bpy.props import EnumProperty, FloatProperty
from bpy.types import PropertyGroup

from ..core.constants import GEOM_BOX, GEOM_CYLINDER, GEOM_MESH, GEOM_SPHERE

PROP_GEOM = "linkforge_geom"


def on_collision_quality_update(self: GeomPropertyGroup, _context: bpy.types.Context) -> None:
    collision_obj = getattr(self, "id_data", None)
    if not collision_obj:
        return

    # Only applies to collision objects
    if self.geom_role != "COLLISION" and "collision" not in collision_obj.name.lower():
        return

    # The parent object should be the link frame
    link_obj = collision_obj.parent
    if not link_obj:
        return

    from ..operators.link_ops import update_collision_quality_realtime

    update_collision_quality_realtime(link_obj, collision_obj)


class GeomPropertyGroup(PropertyGroup):
    """Properties stored directly on each visual/collision mesh object."""

    geom_role: EnumProperty(  # type: ignore
        name="Geometry Role",
        description="Whether this is visual or collision geometry. Set automatically at assignment.",
        items=[
            ("AUTO", "Auto (By Name)", "Determine from name suffix (_visual/_collision)"),
            ("VISUAL", "Visual", "Export as Visual geometry"),
            ("COLLISION", "Collision", "Export as Collision geometry"),
        ],
        default="AUTO",
    )

    geometry_type: EnumProperty(  # type: ignore
        name="Geometry Type",
        description="How this mesh will be exported. Set at assignment time.",
        items=[
            (GEOM_BOX, "Box", "Export as <box> primitive"),
            (GEOM_CYLINDER, "Cylinder", "Export as <cylinder> primitive"),
            (GEOM_SPHERE, "Sphere", "Export as <sphere> primitive"),
            (GEOM_MESH, "Mesh", "Export as mesh file"),
        ],
        default=GEOM_MESH,
    )

    collision_quality: FloatProperty(  # type: ignore
        name="Collision Quality",
        description="Mesh detail preserved (100% = full detail, lower = faster physics)",
        default=50.0,
        min=1.0,
        max=100.0,
        precision=0,
        subtype="PERCENTAGE",
        update=on_collision_quality_update,
    )


def register() -> None:
    """Register property group."""
    try:
        bpy.utils.register_class(GeomPropertyGroup)
    except ValueError:
        bpy.utils.unregister_class(GeomPropertyGroup)
        bpy.utils.register_class(GeomPropertyGroup)

    setattr(
        bpy.types.Object,
        PROP_GEOM,
        typing.cast(typing.Any, bpy.props.PointerProperty(type=GeomPropertyGroup)),
    )


def unregister() -> None:
    """Unregister property group."""
    import contextlib

    with contextlib.suppress(AttributeError):
        delattr(bpy.types.Object, PROP_GEOM)

    with contextlib.suppress(RuntimeError):
        bpy.utils.unregister_class(GeomPropertyGroup)
