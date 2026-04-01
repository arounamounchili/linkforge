"""Blender Property Groups for robot transmissions.

These properties are stored on Empty objects and define transmission characteristics
for ros2_control integration.
"""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, PointerProperty, StringProperty
from bpy.types import Context, PropertyGroup

from ..utils.property_helpers import find_property_owner


def get_transmission_name(self: TransmissionPropertyGroup) -> str:
    """Getter for transmission_name.

    Args:
        self: The property group instance.

    Returns:
        The sanitized Blender object name.
    """
    if not self.id_data:
        return ""

    from .link_props import sanitize_urdf_name

    return sanitize_urdf_name(self.id_data.name)


def set_transmission_name(self: TransmissionPropertyGroup, value: str) -> None:
    """Setter for transmission_name.

    Args:
        self: The property group instance.
        value: The new name to set.
    """
    if not value or not self.id_data:
        return

    from .link_props import sanitize_urdf_name

    sanitized_name = sanitize_urdf_name(value)

    if self.id_data.name != sanitized_name:
        self.id_data.name = sanitized_name


def update_transmission_hierarchy(self: TransmissionPropertyGroup, context: Context) -> None:
    """Update Blender object hierarchy when joint changes.

    Automatically reparents transmission to new joint and moves to joint's collection.
    This ensures visual hierarchy matches logical structure.

    Args:
        self: The property group instance.
        context: The current Blender context.
    """
    # Find the transmission object that owns this property
    transmission_obj = find_property_owner(context, self, "linkforge_transmission")
    if transmission_obj is None or not self.is_robot_transmission:
        return

    # Determine which joint to use based on transmission type
    joint_obj = self.joint1_name if self.transmission_type == "DIFFERENTIAL" else self.joint_name

    # Reparent transmission to the joint
    if joint_obj:
        transmission_obj.parent = joint_obj
        # STRICT ALIGNMENT:
        # We want the transmission to be exactly at the joint origin.
        transmission_obj.matrix_parent_inverse.identity()

        # Reset local position to be at joint origin
        transmission_obj.location = (0, 0, 0)
        transmission_obj.rotation_euler = (0, 0, 0)

        # Move transmission to same collection as joint (for clean organization)
        from ..utils.scene_utils import sync_object_collections

        sync_object_collections(transmission_obj, joint_obj)
    elif transmission_obj.parent:
        # Clear parent (unparent transmission) while preserving world position
        from ..utils.transform_utils import clear_parent_keep_transform

        clear_parent_keep_transform(transmission_obj)


def poll_robot_joint(_self: TransmissionPropertyGroup, obj: bpy.types.Object) -> bool:
    """Filter to only allow robot joint objects in pointer selection.

    Args:
        self: The property group instance.
        obj: The object to check.

    Returns:
        True if the object is a valid robot joint.
    """
    return bool(obj and hasattr(obj, "linkforge_joint") and obj.linkforge_joint.is_robot_joint)


class TransmissionPropertyGroup(PropertyGroup):
    """Properties for a robot transmission stored on an Empty object."""

    # Transmission identification
    is_robot_transmission: BoolProperty(  # type: ignore
        name="Is Robot Transmission",
        description="Mark this Empty as a robot transmission",
        default=False,
    )

    transmission_name: StringProperty(  # type: ignore
        name="Transmission Name",
        description="Name of the transmission in URDF (must be unique)",
        maxlen=64,
        get=get_transmission_name,
        set=set_transmission_name,
    )

    # Transmission type
    transmission_type: EnumProperty(  # type: ignore
        name="Transmission Type",
        description="Type of transmission mechanism",
        items=[
            ("SIMPLE", "Simple", "1:1 joint to actuator transmission"),
            ("DIFFERENTIAL", "Differential", "2 joints to 2 actuators (differential drive)"),
            ("FOUR_BAR_LINKAGE", "Four-Bar Linkage", "Four-bar linkage transmission"),
            ("CUSTOM", "Custom", "Custom transmission type"),
        ],
        default="SIMPLE",
    )

    # Custom type (when transmission_type is CUSTOM)
    custom_type: StringProperty(  # type: ignore
        name="Custom Type",
        description="Custom transmission type identifier",
        default="",
        maxlen=128,
    )

    # Joint selection (for simple transmission)
    joint_name: PointerProperty(  # type: ignore
        name="Joint",
        description="Joint controlled by this transmission",
        type=bpy.types.Object,
        poll=poll_robot_joint,
        update=update_transmission_hierarchy,
    )

    # Joint selection (for differential transmission)
    joint1_name: PointerProperty(  # type: ignore
        name="Joint 1",
        description="First joint in differential transmission",
        type=bpy.types.Object,
        poll=poll_robot_joint,
        update=update_transmission_hierarchy,
    )

    joint2_name: PointerProperty(  # type: ignore
        name="Joint 2",
        description="Second joint in differential transmission",
        type=bpy.types.Object,
        poll=poll_robot_joint,
    )

    # Hardware interface
    hardware_interface: EnumProperty(  # type: ignore
        name="Hardware Interface",
        description="Control interface type for ROS2 Control",
        items=[
            ("POSITION", "Position", "Position control interface"),
            ("VELOCITY", "Velocity", "Velocity control interface"),
            ("EFFORT", "Effort", "Effort/torque control interface"),
        ],
        default="POSITION",
    )

    # Mechanical properties
    mechanical_reduction: FloatProperty(  # type: ignore
        name="Mechanical Reduction",
        description="Gear reduction ratio (actuator/joint)",
        default=1.0,
        min=0.001,
        soft_max=1000.0,
        precision=3,
    )

    offset: FloatProperty(  # type: ignore
        name="Offset",
        description="Joint offset in radians or meters",
        default=0.0,
        precision=5,
    )

    # Actuator naming
    use_custom_actuator_name: BoolProperty(  # type: ignore
        name="Custom Actuator Name",
        description="Use custom actuator name instead of auto-generated",
        default=False,
    )

    actuator_name: StringProperty(  # type: ignore
        name="Actuator Name",
        description="Name of the actuator (motor)",
        default="",
        maxlen=64,
    )

    # For differential transmissions
    actuator1_name: StringProperty(  # type: ignore
        name="Actuator 1 Name",
        description="Name of the first actuator",
        default="",
        maxlen=64,
    )

    actuator2_name: StringProperty(  # type: ignore
        name="Actuator 2 Name",
        description="Name of the second actuator",
        default="",
        maxlen=64,
    )


# Registration
def register() -> None:
    """Register property group."""
    try:
        bpy.utils.register_class(TransmissionPropertyGroup)
    except ValueError:
        # If already registered (e.g. from reload), unregister first to ensure clean state
        bpy.utils.unregister_class(TransmissionPropertyGroup)
        bpy.utils.register_class(TransmissionPropertyGroup)

    bpy.types.Object.linkforge_transmission = PointerProperty(type=TransmissionPropertyGroup)  # type: ignore


def unregister() -> None:
    """Unregister property group."""
    import contextlib

    with contextlib.suppress(AttributeError):
        del bpy.types.Object.linkforge_transmission  # type: ignore

    with contextlib.suppress(RuntimeError):
        bpy.utils.unregister_class(TransmissionPropertyGroup)


if __name__ == "__main__":
    register()
