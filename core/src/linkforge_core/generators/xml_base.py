"""Base XML generator shared across URDF, XACRO, SDF, MJCF."""

from __future__ import annotations

__all__ = ["RobotXMLGenerator"]

import xml.etree.ElementTree as ET
from functools import singledispatchmethod
from pathlib import Path
from typing import Any

from ..base import RobotGenerator
from ..models.geometry import Box, Cylinder, Mesh, Sphere, Transform
from ..models.link import Inertial
from ..utils.math_utils import format_float
from ..utils.path_utils import get_export_path


class RobotXMLGenerator(RobotGenerator[str]):
    """Abstract base class for XML-based robotics format generators."""

    def __init__(self, pretty_print: bool = True, output_path: Path | None = None) -> None:
        """Initialize base XML generator.

        Args:
            pretty_print: If True, format XML with indentation
            output_path: Base path to calculate relative mesh paths
        """
        self.pretty_print = pretty_print
        self.output_path = output_path
        self.global_materials: dict[str, Any] = {}

    def _format_value(self, value: Any) -> str:
        """Format a basic float or integer value to a string.

        This acts as a hook for generators like XACRO to check for property
        substitution before defaulting to standard string formatting.

        Args:
            value: The numeric value or standard object to format.

        Returns:
            The string representation.
        """
        if isinstance(value, (float, int)) and not isinstance(value, bool):
            return format_float(float(value))
        return str(value)

    def _format_vector(self, x: float, y: float, z: float) -> str:
        """Format a 3D vector.

        Args:
            x: X component.
            y: Y component.
            z: Z component.

        Returns:
            String representation of vector.
        """
        return f"{self._format_value(x)} {self._format_value(y)} {self._format_value(z)}"

    def _add_origin_element(
        self, parent: ET.Element, transform: Transform, tag: str = "origin"
    ) -> None:
        """Add origin element to parent."""
        if transform is None or transform == Transform.identity():
            return

        xyz_str = self._format_vector(transform.xyz.x, transform.xyz.y, transform.xyz.z)
        rpy_str = self._format_vector(transform.rpy.x, transform.rpy.y, transform.rpy.z)
        ET.SubElement(parent, tag, xyz=xyz_str, rpy=rpy_str)

    def _add_inertial_element(
        self, parent: ET.Element, inertial: Inertial, tag: str = "inertial"
    ) -> None:
        """Add inertial element to parent."""
        inertial_elem = ET.SubElement(parent, tag)

        # Origin (COM position)
        self._add_origin_element(inertial_elem, inertial.origin)

        # Mass
        ET.SubElement(inertial_elem, "mass", value=self._format_value(inertial.mass))

        # Inertia tensor
        inertia = inertial.inertia
        ET.SubElement(
            inertial_elem,
            "inertia",
            ixx=self._format_value(inertia.ixx),
            ixy=self._format_value(inertia.ixy),
            ixz=self._format_value(inertia.ixz),
            iyy=self._format_value(inertia.iyy),
            iyz=self._format_value(inertia.iyz),
            izz=self._format_value(inertia.izz),
        )

    @singledispatchmethod
    def _add_geometry_element(
        self, _geometry: Any, parent: ET.Element, tag: str = "geometry"
    ) -> None:
        """Add geometry element to parent. Overridden for specific types."""
        # Fallback for unsupported geometry: creates empty container
        ET.SubElement(parent, tag)

    @_add_geometry_element.register
    def _(self, geometry: Box, parent: ET.Element, tag: str = "geometry") -> None:
        geom_elem = ET.SubElement(parent, tag)
        size_str = self._format_vector(geometry.size.x, geometry.size.y, geometry.size.z)
        ET.SubElement(geom_elem, "box", size=size_str)

    @_add_geometry_element.register
    def _(self, geometry: Cylinder, parent: ET.Element, tag: str = "geometry") -> None:
        geom_elem = ET.SubElement(parent, tag)
        ET.SubElement(
            geom_elem,
            "cylinder",
            radius=self._format_value(geometry.radius),
            length=self._format_value(geometry.length),
        )

    @_add_geometry_element.register
    def _(self, geometry: Sphere, parent: ET.Element, tag: str = "geometry") -> None:
        geom_elem = ET.SubElement(parent, tag)
        ET.SubElement(geom_elem, "sphere", radius=self._format_value(geometry.radius))

    @_add_geometry_element.register
    def _(self, geometry: Mesh, parent: ET.Element, tag: str = "geometry") -> None:
        geom_elem = ET.SubElement(parent, tag)

        output_dir = self.output_path.parent if self.output_path else None
        export_path = get_export_path(geometry.resource, relative_to=output_dir)

        attrib: dict[str, str | None] = {"filename": export_path}

        # Check if scale is not default (1.0, 1.0, 1.0)
        if geometry.scale.x != 1.0 or geometry.scale.y != 1.0 or geometry.scale.z != 1.0:
            attrib["scale"] = self._format_vector(
                geometry.scale.x, geometry.scale.y, geometry.scale.z
            )

        self._create_element(geom_elem, "mesh", **attrib)

    def _create_element(
        self, parent: ET.Element, tag: str, **kwargs: str | float | int | bool | None
    ) -> ET.Element:
        """Create an XML element, stripping None values and converting types to str.

        Args:
            parent: Parent XML element
            tag: Tag name for the new element
            **kwargs: Attributes for the new element

        Returns:
            The newly created XML element
        """
        # Convert all values to strings and strip None values
        attrib = {k: self._format_value(v) for k, v in kwargs.items() if v is not None}
        return ET.SubElement(parent, tag, attrib)
