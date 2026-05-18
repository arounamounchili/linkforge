"""Protocols for the LinkForge Composer layer.

These protocols define the interfaces that allow different builders to
communicate without tight coupling.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from ..models.robot import Robot
    from .link_builder import LinkBuilder


class IComposer(Protocol):
    """Interface for a builder that can contain and manage links."""

    robot: Robot
    _active_link_builders: list[LinkBuilder]
    _parent_stack: list[str]

    def link(
        self, name: str, parent: str | None = None, joint_name: str | None = None
    ) -> LinkBuilder:
        """Start building a new link."""
        ...

    def material(
        self, name: str, color: tuple[float, float, float, float] | None = None
    ) -> IComposer:
        """Define a global material."""
        ...

    def build(self, validate: bool = True) -> Robot:
        """Finalize the assembly and return the completed Robot model."""
        ...
