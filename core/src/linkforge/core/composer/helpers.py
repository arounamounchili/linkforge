"""Geometry primitive factory helpers for LinkForge Composer."""

from __future__ import annotations

from ..models.geometry import Box, Cylinder, Mesh, Sphere, Vector3


def box(x: float, y: float, z: float) -> Box:
    """Helper to create Box geometry."""
    return Box(size=Vector3(x, y, z))


def cylinder(radius: float, length: float) -> Cylinder:
    """Helper to create Cylinder geometry."""
    return Cylinder(radius=radius, length=length)


def sphere(radius: float) -> Sphere:
    """Helper to create Sphere geometry."""
    return Sphere(radius=radius)


def mesh(resource: str, scale: tuple[float, float, float] = (1.0, 1.0, 1.0)) -> Mesh:
    """Helper to create Mesh geometry."""
    return Mesh(resource=resource, scale=Vector3(*scale))
