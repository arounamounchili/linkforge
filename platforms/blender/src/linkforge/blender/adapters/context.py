"""Abstraction layer for Blender host environment.

This module defines the IBlenderContext protocol, which decouples LinkForge
from direct bpy dependencies, enabling pure unit testing and multi-platform
support.
"""

from __future__ import annotations

import typing
from collections.abc import Iterable
from typing import Any, runtime_checkable


@runtime_checkable
class IBlenderContext(typing.Protocol):
    """Protocol defining the necessary Blender environment access."""

    @property
    def scene(self) -> Any:
        """Active Blender scene."""
        ...

    @property
    def data(self) -> Any:
        """Access to bpy.data."""
        ...

    @property
    def ops(self) -> Any:
        """Access to bpy.ops."""
        ...

    @property
    def view_layer(self) -> Any:
        """Active Blender view layer."""
        ...

    @property
    def active_object(self) -> Any | None:
        """Currently active Blender object."""
        ...

    @property
    def preferences(self) -> Any:
        """Blender user preferences."""
        ...

    @property
    def window_manager(self) -> Any:
        """Blender window manager."""
        ...

    def get_objects(self) -> Iterable[Any]:
        """Retrieve all objects relevant for the current context."""
        ...

    def get_active_object(self) -> Any | None:
        """Retrieve the currently active object."""
        ...


class BlenderContext:
    """Real-world implementation of IBlenderContext using live bpy."""

    def __init__(self, bpy_instance: Any = None):
        """Initialize with a specific bpy instance (defaults to global bpy)."""
        import bpy

        self._global_bpy = bpy
        if bpy_instance is None:
            self._bpy = bpy
        else:
            self._bpy = bpy_instance

    @property
    def _ctx(self) -> Any:
        """Internal helper to get the active context object."""
        # If we are holding the bpy module, return bpy.context
        # Otherwise, assume we are holding a context object directly
        if hasattr(self._bpy, "context"):
            return self._bpy.context
        return self._bpy

    @property
    def scene(self) -> Any:
        """Return the active scene from context."""
        return self._ctx.scene

    @property
    def data(self) -> Any:
        """Return the data block."""
        if hasattr(self._bpy, "data"):
            return self._bpy.data
        return self._global_bpy.data

    @property
    def ops(self) -> Any:
        """Return the operators block."""
        if hasattr(self._bpy, "ops"):
            return self._bpy.ops
        return self._global_bpy.ops

    @property
    def view_layer(self) -> Any:
        """Return the active view layer."""
        return self._ctx.view_layer

    @property
    def active_object(self) -> Any | None:
        """Return the active object from context."""
        return self._ctx.active_object

    @property
    def preferences(self) -> Any:
        """Return the user preferences."""
        return self._ctx.preferences

    @property
    def window_manager(self) -> Any:
        """Return the window manager."""
        return self._ctx.window_manager

    def get_objects(self) -> Iterable[Any]:
        """Return all objects in the current scene."""
        return typing.cast(Iterable[Any], self.data.objects)

    def get_active_object(self) -> Any | None:
        """Return the active object from context."""
        return self.active_object
