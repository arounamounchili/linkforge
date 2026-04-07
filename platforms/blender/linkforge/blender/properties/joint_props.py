"""Blender Property Groups for robot joints.

These properties are stored on Empty objects and define joint characteristics.
"""

from __future__ import annotations

import typing

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import Context, PropertyGroup

if typing.TYPE_CHECKING:
    from .link_props import LinkPropertyGroup

from linkforge_core.utils.string_utils import sanitize_name as sanitize_urdf_name

from linkforge.blender.utils.property_helpers import find_property_owner
from linkforge.blender.utils.scene_utils import clear_stats_cache


def get_joint_name(self: JointPropertyGroup) -> str:
    """Getter for joint_name - returns the persistent URDF identity.

    Args:
        self: The JointPropertyGroup instance.

    Returns:
        The sanitized URDF name.
    """
    # Prioritize the stored identity to avoid Blender's .001 suffixing
    if self.urdf_name_stored:
        return str(self.urdf_name_stored)

    if not self.id_data:
        return ""

    return sanitize_urdf_name(str(self.id_data.name))


def set_joint_name(self: JointPropertyGroup, value: str) -> None:
    """Setter for joint_name - updates persistent identity and object name.

    Args:
        self: The JointPropertyGroup instance.
        value: The new name value to set.
    """
    if not value or not self.id_data:
        return

    # Sanitize joint name for URDF
    sanitized_name = sanitize_urdf_name(value)

    # Store the persistent identity
    self.urdf_name_stored = sanitized_name

    # Update object name to match joint name
    # Blender will handle collisions by appending suffixes, but our stored name persists
    if self.id_data.name != sanitized_name:
        self.id_data.name = sanitized_name

    # Clear statistics cache when name changes
    clear_stats_cache()


def update_joint_hierarchy(self: JointPropertyGroup, context: Context) -> None:
    """Update Blender object hierarchy when parent/child links change.

    Establishes hierarchy: parent_link → joint → child_link
    This matches URDF import behavior and shows kinematic tree in outliner.
    """
    if not bpy:
        return

    # Find the joint object that owns this property
    joint_obj = find_property_owner(context, self, "linkforge_joint")
    if joint_obj is None or not self.is_robot_joint:
        return

    from linkforge.blender.utils.transform_utils import (
        clear_parent_keep_transform,
        set_parent_keep_transform,
    )

    # Parent-child objects directly from pointers
    parent_obj = self.parent_link
    child_obj = self.child_link

    # Handle parent link hierarchy
    if parent_obj:
        # Parent the Joint to the Parent Link
        set_parent_keep_transform(joint_obj, parent_obj)

        # Move to parent's collection (organization)
        from linkforge.blender.utils.scene_utils import sync_object_collections

        sync_object_collections(joint_obj, parent_obj)
    elif joint_obj.parent:
        # Clear parent (unparent joint) while preserving world position
        clear_parent_keep_transform(joint_obj)

    # Handle child link hierarchy
    if child_obj:
        # Parent the Child Link to the Joint
        set_parent_keep_transform(child_obj, joint_obj)
    else:
        # Find and unparent any child link that was parented to this joint
        scene = context.scene
        if scene:
            for obj in scene.objects:
                if (
                    obj.parent == joint_obj
                    and hasattr(obj, "linkforge")
                    and typing.cast("LinkPropertyGroup", obj.linkforge).is_robot_link
                ):
                    # Clear parent while preserving world position
                    clear_parent_keep_transform(obj)
                    break  # Only unparent one child

    # Clear statistics cache when hierarchy changes
    clear_stats_cache(self, context)


def poll_robot_link(_self: JointPropertyGroup, obj: bpy.types.Object) -> bool:
    """Filter to only allow robot link objects in pointer selection."""
    return bool(hasattr(obj, "linkforge") and obj.linkforge.is_robot_link)


def poll_robot_joint(self: JointPropertyGroup, obj: bpy.types.Object) -> bool:
    """Filter to only allow other robot joint objects in pointer selection."""
    if not obj or obj.type != "EMPTY":
        return False

    joint_props = getattr(obj, "linkforge_joint", None)
    if not joint_props or not joint_props.is_robot_joint:
        return False

    # Prevent self-mimicry
    # We compare the objects that own the properties
    # find_property_owner is imported at the top
    current_obj = find_property_owner(bpy.context, self, "linkforge_joint")
    return bool(obj != current_obj)


class JointPropertyGroup(PropertyGroup):
    """Properties for a robot joint stored on an Empty object."""

    # Joint identification
    is_robot_joint: BoolProperty(  # type: ignore
        name="Is Robot Joint",
        description="Mark this Empty as a robot joint",
        default=False,
    )

    # Persistent URDF Identity
    # Decouples logical URDF naming from physical Blender object names (resilient to .001 suffixes)
    urdf_name_stored: StringProperty(  # type: ignore
        name="URDF Name",
        description="Persistent URDF name. Prevents mapping breakage if Blender renames the object",
        default="",
    )

    joint_name: StringProperty(  # type: ignore
        name="Joint Name",
        description="Name of the joint in URDF (must be unique)",
        maxlen=64,
        get=get_joint_name,
        set=set_joint_name,
        update=clear_stats_cache,
    )

    # Joint type
    joint_type: EnumProperty(  # type: ignore
        name="Joint Type",
        description="Type of joint connection",
        items=[
            ("REVOLUTE", "Revolute", "Rotates around axis with limits"),
            ("CONTINUOUS", "Continuous", "Rotates around axis without limits"),
            ("PRISMATIC", "Prismatic", "Slides along axis with limits"),
            ("FIXED", "Fixed", "No motion allowed"),
            ("FLOATING", "Floating", "6 DOF free in space"),
            ("PLANAR", "Planar", "2D motion in a plane"),
        ],
        default="REVOLUTE",
        update=clear_stats_cache,
    )

    # Parent and child links
    parent_link: PointerProperty(  # type: ignore
        name="Parent Link",
        description="Link this joint connects from (base side)",
        type=bpy.types.Object,
        poll=poll_robot_link,
        update=update_joint_hierarchy,
    )

    child_link: PointerProperty(  # type: ignore
        name="Child Link",
        description="Link this joint connects to (moving side)",
        type=bpy.types.Object,
        poll=poll_robot_link,
        update=update_joint_hierarchy,
    )

    # Joint axis
    axis: EnumProperty(  # type: ignore
        name="Axis",
        description="Which direction the joint moves (rotation or sliding axis)",
        items=[
            ("X", "X", "X axis (red)"),
            ("Y", "Y", "Y axis (green)"),
            ("Z", "Z", "Z axis (blue)"),
            ("CUSTOM", "Custom", "Custom axis direction"),
        ],
        default="Z",
    )

    # Custom axis (when axis is CUSTOM)
    # Note: No min/max limits - values will be automatically normalized to unit vector
    custom_axis_x: FloatProperty(  # type: ignore
        name="Axis X",
        description="Custom axis X component (will be normalized to unit vector)",
        default=0.0,
        soft_min=-10.0,
        soft_max=10.0,
    )

    custom_axis_y: FloatProperty(  # type: ignore
        name="Axis Y",
        description="Custom axis Y component (will be normalized to unit vector)",
        default=0.0,
        soft_min=-10.0,
        soft_max=10.0,
    )

    custom_axis_z: FloatProperty(  # type: ignore
        name="Axis Z",
        description="Custom axis Z component (will be normalized to unit vector)",
        default=1.0,
        soft_min=-10.0,
        soft_max=10.0,
    )

    # Joint limits (for revolute and prismatic)
    use_limits: BoolProperty(  # type: ignore
        name="Use Limits",
        description="Restrict how far the joint can move (safety limits)",
        default=False,
    )

    limit_lower: FloatProperty(  # type: ignore
        name="Lower Limit",
        description="Minimum position (radians for revolute/continuous joints, meters for prismatic joints)",
        default=-3.14159265359,  # -π
        soft_min=-6.28318530718,  # -2π
        soft_max=6.28318530718,  # 2π
    )

    limit_upper: FloatProperty(  # type: ignore
        name="Upper Limit",
        description="Maximum position (radians for revolute/continuous joints, meters for prismatic joints)",
        default=3.14159265359,  # π
        soft_min=-6.28318530718,  # -2π
        soft_max=6.28318530718,  # 2π
    )

    limit_effort: FloatProperty(  # type: ignore
        name="Max Effort",
        description="Maximum force/torque the joint motor can apply",
        default=10.0,
        min=0.0,
        soft_max=100.0,
    )

    limit_velocity: FloatProperty(  # type: ignore
        name="Max Velocity",
        description="Maximum speed the joint can move",
        default=1.0,
        min=0.0,
        soft_max=10.0,
    )

    # Joint dynamics
    use_dynamics: BoolProperty(  # type: ignore
        name="Use Dynamics",
        description="Add friction and damping for realistic motion (optional)",
        default=False,
    )

    dynamics_damping: FloatProperty(  # type: ignore
        name="Damping",
        description="Resistance to motion (slows down movement)",
        default=0.0,
        min=0.0,
        soft_max=10.0,
    )

    dynamics_friction: FloatProperty(  # type: ignore
        name="Friction",
        description="Static friction (resistance to starting motion)",
        default=0.0,
        min=0.0,
        soft_max=10.0,
    )

    # Mimic joint
    use_mimic: BoolProperty(  # type: ignore
        name="Mimic Another Joint",
        description="Make this joint copy another joint's movement (like coupled fingers)",
        default=False,
    )

    mimic_joint: PointerProperty(  # type: ignore
        name="Mimic Joint",
        description="Which joint to copy movement from",
        type=bpy.types.Object,
        poll=poll_robot_joint,
    )

    mimic_multiplier: FloatProperty(  # type: ignore
        name="Multiplier",
        description="Movement scale (2.0 = moves twice as much, 0.5 = half as much)",
        default=1.0,
    )

    mimic_offset: FloatProperty(  # type: ignore
        name="Offset",
        description="Position offset added to mimic joint movement (applied after multiplier)",
        default=0.0,
    )

    # Joint Safety Controller
    use_safety_controller: BoolProperty(  # type: ignore
        name="Use Safety Controller",
        description="Add a safety controller to the joint (standard URDF feature)",
        default=False,
    )

    safety_soft_lower_limit: FloatProperty(  # type: ignore
        name="Soft Lower Limit",
        description="Lower bound of the joint safety controller",
        default=0.0,
    )

    safety_soft_upper_limit: FloatProperty(  # type: ignore
        name="Soft Upper Limit",
        description="Upper bound of the joint safety controller",
        default=0.0,
    )

    safety_k_position: FloatProperty(  # type: ignore
        name="K Position",
        description="Position gain for safety controller",
        default=0.0,
    )

    safety_k_velocity: FloatProperty(  # type: ignore
        name="K Velocity",
        description="Velocity gain for safety controller",
        default=0.0,
    )

    # Joint Calibration
    use_calibration: BoolProperty(  # type: ignore
        name="Use Calibration",
        description="Add calibration settings to the joint",
        default=False,
    )

    calibration_rising: FloatProperty(  # type: ignore
        name="Rising Edge",
        description="Position of the rising edge (optional)",
        default=0.0,
    )

    use_calibration_rising: BoolProperty(  # type: ignore
        name="Specify Rising Edge",
        description="Whether to include the rising edge in calibration",
        default=False,
    )

    calibration_falling: FloatProperty(  # type: ignore
        name="Falling Edge",
        description="Position of the falling edge (optional)",
        default=0.0,
    )

    use_calibration_falling: BoolProperty(  # type: ignore
        name="Specify Falling Edge",
        description="Whether to include the falling edge in calibration",
        default=False,
    )


# Registration
def register() -> None:
    """Register property group."""
    try:
        bpy.utils.register_class(JointPropertyGroup)
    except ValueError:
        # If already registered (e.g. from reload), unregister first to ensure clean state
        bpy.utils.unregister_class(JointPropertyGroup)
        bpy.utils.register_class(JointPropertyGroup)

    prop_name = "linkforge_joint"
    setattr(
        bpy.types.Object,
        prop_name,
        typing.cast(typing.Any, PointerProperty(type=JointPropertyGroup)),
    )


def unregister() -> None:
    """Unregister property group."""
    import contextlib

    with contextlib.suppress(AttributeError):
        delattr(bpy.types.Object, "linkforge_joint")

    with contextlib.suppress(RuntimeError):
        bpy.utils.unregister_class(JointPropertyGroup)


if __name__ == "__main__":
    register()
