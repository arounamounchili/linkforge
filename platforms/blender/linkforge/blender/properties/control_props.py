"""Blender Property Groups for centralized ros2_control configuration.

These properties are stored on the Scene and define the mapping between
robot joints and ros2_control interfaces (command and state).
"""

from __future__ import annotations

import bpy
from bpy.props import BoolProperty, CollectionProperty, EnumProperty, StringProperty
from bpy.types import PropertyGroup


class Ros2ControlParameterProperty(PropertyGroup):
    """Key-value pair for ros2_control parameters."""

    name: StringProperty(  # type: ignore
        name="Key",
        description="Parameter name",
        default="param",
    )
    value: StringProperty(  # type: ignore
        name="Value",
        description="Parameter value",
        default="0.0",
    )


class Ros2ControlInterfaceProperty(PropertyGroup):
    """Property group for a single ros2_control interface."""

    name: EnumProperty(  # type: ignore
        name="Interface",
        description="Interface type",
        items=[
            ("position", "Position", "Position interface"),
            ("velocity", "Velocity", "Velocity interface"),
            ("effort", "Effort", "Effort/Torque interface"),
        ],
        default="position",
    )

    # Parameters for this specific interface (e.g., min/max for command interfaces)
    parameters: CollectionProperty(type=Ros2ControlParameterProperty)  # type: ignore


class Ros2ControlJointProperty(PropertyGroup):
    """Property group for a joint's ros2_control mapping."""

    name: StringProperty(  # type: ignore
        name="Joint Name",
        description="Name of the joint in the kinematic tree",
        default="",
    )

    # Command Interfaces (what we send)
    # Using Booleans for common interfaces for quick UI access
    cmd_position: BoolProperty(name="Position", default=True)  # type: ignore
    cmd_velocity: BoolProperty(name="Velocity", default=False)  # type: ignore
    cmd_effort: BoolProperty(name="Effort", default=False)  # type: ignore

    # State Interfaces (what we read)
    state_position: BoolProperty(name="Position", default=True)  # type: ignore
    state_velocity: BoolProperty(name="Velocity", default=True)  # type: ignore
    state_effort: BoolProperty(name="Effort", default=False)  # type: ignore

    # UI State
    show_parameters: BoolProperty(name="Show Parameters", default=False)  # type: ignore

    # Advanced parameters (rarely used but supported by spec)
    parameters: CollectionProperty(type=Ros2ControlParameterProperty)  # type: ignore


def register() -> None:
    """Register property groups."""
    for cls in [
        Ros2ControlParameterProperty,
        Ros2ControlInterfaceProperty,
        Ros2ControlJointProperty,
    ]:
        try:
            bpy.utils.register_class(cls)
        except ValueError:
            bpy.utils.unregister_class(cls)
            bpy.utils.register_class(cls)


def unregister() -> None:
    """Unregister property groups."""
    bpy.utils.unregister_class(Ros2ControlJointProperty)
    bpy.utils.unregister_class(Ros2ControlInterfaceProperty)
    bpy.utils.unregister_class(Ros2ControlParameterProperty)


if __name__ == "__main__":
    register()
