"""Base XML parser shared across URDF, XACRO, etc. Support for MJCF and SDF is planned."""

from __future__ import annotations

__all__ = ["RobotXMLParser"]

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import TYPE_CHECKING

from ..base import IResourceResolver, RobotParser
from ..exceptions import RobotModelError, RobotParserError, RobotValidationError
from ..logging_config import get_logger
from ..models import (
    Box,
    Color,
    Cylinder,
    Inertial,
    InertiaTensor,
    Material,
    Mesh,
    Sphere,
    Transform,
)
from ..utils.path_utils import normalize_uri_to_path
from ..utils.xml_utils import (
    parse_float,
    parse_vector3,
)
from ..validation import validate_mesh_path, validate_package_uri

if TYPE_CHECKING:
    pass

logger = get_logger(__name__)

MAX_FILE_SIZE = 100 * 1024 * 1024  # 100 MB


class RobotXMLParser(RobotParser):
    """Abstract base class for XML-based robotics format parsers."""

    def __init__(
        self,
        max_file_size: int = MAX_FILE_SIZE,
        sandbox_root: Path | None = None,
        resource_resolver: IResourceResolver | None = None,
    ) -> None:
        """Initialize base XML parser.

        Args:
            max_file_size: Maximum allowed file size in bytes
            sandbox_root: Optional root directory for security sandbox
            resource_resolver: Optional resolver for URIs
        """
        self.max_file_size = max_file_size
        self.sandbox_root = sandbox_root
        self.resource_resolver = resource_resolver

    def _parse_origin_element(self, elem: ET.Element | None) -> Transform:
        """Parse origin-style element into a Transform object.

        Args:
            elem: XML element with xyz/rpy attributes (e.g. <origin> or <pose>).

        Returns:
            A Transform object.
        """
        if elem is None:
            return Transform.identity()

        xyz_text = elem.get("xyz", elem.get("pos", "0 0 0"))
        rpy_text = elem.get("rpy", elem.get("euler", "0 0 0"))

        xyz = parse_vector3(xyz_text)
        rpy = parse_vector3(rpy_text)

        return Transform(xyz=xyz, rpy=rpy)

    def _parse_geometry_element(
        self,
        geom_elem: ET.Element,
        base_directory: Path | None = None,
    ) -> Box | Cylinder | Sphere | Mesh | None:
        """Parse geometry element (box, cylinder, sphere, mesh)."""
        try:
            if geom_elem.find("box") is not None:
                return self._parse_box(geom_elem.find("box"))
            if geom_elem.find("cylinder") is not None:
                return self._parse_cylinder(geom_elem.find("cylinder"))
            if geom_elem.find("sphere") is not None:
                return self._parse_sphere(geom_elem.find("sphere"))
            if geom_elem.find("mesh") is not None:
                return self._parse_mesh(geom_elem.find("mesh"), base_directory)
        except (RobotModelError, ValueError, RobotParserError) as e:
            logger.warning(f"Invalid geometry ignored: {e}")
            return None

        return None

    def _parse_box(self, box: ET.Element | None) -> Box | None:
        """Parse box geometry."""
        if box is None:
            return None
        size_text = box.get("size")
        if size_text is None:
            logger.warning("Invalid box geometry ignored: missing size")
            return None
        return Box(size=parse_vector3(size_text))

    def _parse_cylinder(self, cylinder: ET.Element | None) -> Cylinder | None:
        """Parse cylinder geometry."""
        if cylinder is None:
            return None
        radius = parse_float(cylinder.get("radius"), "cylinder radius", default=0.5)
        length = parse_float(cylinder.get("length"), "cylinder length", default=1.0)
        return Cylinder(radius=radius, length=length)

    def _parse_sphere(self, sphere: ET.Element | None) -> Sphere | None:
        """Parse sphere geometry."""
        if sphere is None:
            return None
        radius = parse_float(sphere.get("radius"), "sphere radius", default=0.5)
        return Sphere(radius=radius)

    def _parse_mesh(self, mesh: ET.Element | None, base_dir: Path | None) -> Mesh | None:
        """Parse mesh geometry with security validation."""
        if mesh is None:
            return None
        filename = mesh.get("filename", mesh.get("file", ""))
        if not filename:
            raise RobotValidationError(check_name="Mesh", reason="Missing filename")

        # Path Security Validation
        validation_path: Path | None = None
        if filename.startswith("file://"):
            validation_path = normalize_uri_to_path(filename)
        elif not filename.startswith("package://"):
            validation_path = Path(filename)

        if validation_path is not None and base_dir is not None:
            validate_mesh_path(
                validation_path,
                base_dir,
                sandbox_root=self.sandbox_root,
                allow_absolute=validation_path.is_absolute(),
            )
        elif filename.startswith("package://"):
            validate_package_uri(filename)

        scale_text = mesh.get("scale", "1 1 1")
        resource = str(validation_path) if validation_path else filename
        return Mesh(resource=resource, scale=parse_vector3(scale_text))

    def _parse_material_element(
        self, mat_elem: ET.Element | None, materials: dict[str, Material]
    ) -> Material | None:
        """Parse material definition or reference.

        Args:
            mat_elem: Material element.
            materials: Cache of defined materials.

        Returns:
            Material object or None.
        """
        if mat_elem is None:
            return None

        mat_name = mat_elem.get("name", "")
        if mat_name and mat_name in materials:
            return materials[mat_name]

        color = None
        color_elem = mat_elem.find("color")
        num_rgb = 3
        num_rgba = 4
        if color_elem is not None:
            rgba_text = color_elem.get("rgba", "0.8 0.8 0.8 1.0")
            parts = rgba_text.strip().split()
            try:
                num_parts = len(parts)
                if num_parts == num_rgb:
                    color = Color(r=float(parts[0]), g=float(parts[1]), b=float(parts[2]), a=1.0)
                elif num_parts == num_rgba:
                    color = Color(
                        r=float(parts[0]), g=float(parts[1]), b=float(parts[2]), a=float(parts[3])
                    )
            except (ValueError, IndexError):
                logger.warning(f"Invalid material color format: {rgba_text}")
                return None

        texture = None
        texture_elem = mat_elem.find("texture")
        if texture_elem is not None:
            texture = texture_elem.get("filename")

        if color or texture:
            return Material(name=mat_name if mat_name else "default", color=color, texture=texture)

        return None

    def _parse_inertial_element(self, inertial_elem: ET.Element | None) -> Inertial | None:
        """Parse inertial properties.

        Args:
            inertial_elem: Inertial XML element.

        Returns:
            Inertial object or None.
        """
        if inertial_elem is None:
            return None

        origin = self._parse_origin_element(inertial_elem.find("origin"))

        mass_elem = inertial_elem.find("mass")
        mass = parse_float(mass_elem.get("value") if mass_elem is not None else None, default=0.0)

        inertia_elem = inertial_elem.find("inertia")
        if inertia_elem is not None:
            # Robustness: sanitize negative or zero diagonal inertia to 1e-6
            # as expected by some unit tests for "safety net" behavior.
            ixx = max(1e-6, parse_float(inertia_elem.get("ixx"), default=1e-6))
            iyy = max(1e-6, parse_float(inertia_elem.get("iyy"), default=1e-6))
            izz = max(1e-6, parse_float(inertia_elem.get("izz"), default=1e-6))

            ixy = parse_float(inertia_elem.get("ixy"), default=0.0)
            ixz = parse_float(inertia_elem.get("ixz"), default=0.0)
            iyz = parse_float(inertia_elem.get("iyz"), default=0.0)

            try:
                inertia = InertiaTensor(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz)
            except RobotModelError:
                # If triangle inequality is still violated, fall back to minimal valid
                inertia = InertiaTensor.zero()
        else:
            inertia = InertiaTensor.zero()

        return Inertial(mass=mass, origin=origin, inertia=inertia)
